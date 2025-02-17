// Copyright (c) 2024 - 2025 Fraunhofer IOSB and contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Fraunhofer IOSB nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "../include/multisensor_calibration/calibration/ExtrinsicCameraLidarCalibrationNodelet.h"

// Std
#include <algorithm>
#include <cstring>
#include <fstream>
#include <functional>
#include <future>
#include <random>
#include <thread>

// Qt
#include <QFile>

// ROS
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/CameraInfo.h>

// PCL
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>

// Eigen
#include <Eigen/Eigenvalues>

// multisensor_calibration
#include "../../include/multisensor_calibration/common/common.h"
#include "../../include/multisensor_calibration/common/utils.hpp"
#include <multisensor_calibration/LidarDataProcessingNodeletConfig.h>

namespace multisensor_calibration
{

//==================================================================================================
ExtrinsicCameraLidarCalibrationNodelet::ExtrinsicCameraLidarCalibrationNodelet() :
  Extrinsic2d3dCalibrationBase<CameraDataProcessor, LidarDataProcessor>(EXTRINSIC_CAMERA_LIDAR_CALIBRATION),
  pImgCloudApproxSync_(nullptr),
  pImgCloudExactSync_(nullptr),
  syncQueueSize_(DEFAULT_SYNC_QUEUE_SIZE),
  useExactSync_(false),
  pFrustumCullingFilters_()
{
    //--- connect dyn reconfigure callback
    dynConfigServer_.setCallback(
      boost::bind(&ExtrinsicCameraLidarCalibrationNodelet::dynReconfigureCallback, this, _1, _2));
}

//==================================================================================================
ExtrinsicCameraLidarCalibrationNodelet::~ExtrinsicCameraLidarCalibrationNodelet()
{
    //--- reset pointers message filters before sensor processors in order to avoid seg fault during
    //--- disconnection of callbacks.
    pImgCloudApproxSync_.reset();
    pImgCloudExactSync_.reset();
    pCamDataProcessor_.reset();
    pLidarDataProcessor_.reset();
}

//==================================================================================================
void ExtrinsicCameraLidarCalibrationNodelet::calibrateLastObservation()
{
    if (!pCamDataProcessor_->isCameraIntrinsicsSet())
    {
        ROS_ERROR("[%s] Could not calibrate last observation. Camera intrinsics are not set",
                  nodeletName_.c_str());
        return;
    }

    //--- check that for this calibration iteration, both camera and lidar observations are available
    //--- i.e. if num of captured observations is smaller than current calibration iteration, return
    if (pCamDataProcessor_->getNumCalibIterations() < calibrationItrCnt_ ||
        pLidarDataProcessor_->getNumCalibIterations() < calibrationItrCnt_)
        return;

    //--- get last observations from camera
    std::set<uint> cameraObservationIds;
    std::vector<cv::Point2f> cameraCornerObservations;
    pCamDataProcessor_->getOrderedObservations(cameraObservationIds, cameraCornerObservations,
                                               calibrationItrCnt_, 1);

    //--- get last observations from LiDAR
    std::set<uint> lidarObservationIds;
    std::vector<cv::Point3f> lidarCornerObservations;
    pLidarDataProcessor_->getOrderedObservations(lidarObservationIds, lidarCornerObservations,
                                                 calibrationItrCnt_, 1);

    //--- remove observations that do not have a correspondence in the other list
    removeCornerObservationsWithoutCorrespondence(cameraObservationIds,
                                                  lidarObservationIds,
                                                  lidarCornerObservations);
    removeCornerObservationsWithoutCorrespondence(lidarObservationIds,
                                                  cameraObservationIds,
                                                  cameraCornerObservations);

    //--- do pnp calibration for a single pose of calibration target.
    //--- use pose guess if there has already been a calibration, to enforce consistency between
    //--- calibrations.
    lib3d::Extrinsics newExtrinsics; // new (temporary) sensor extrinsics after pnp calibration
    std::pair<double, int> pnpRetVal = runPnp(
      cameraCornerObservations.cbegin(), cameraCornerObservations.cend(),
      lidarCornerObservations.cbegin(), lidarCornerObservations.cend(),
      pCamDataProcessor_->cameraIntrinsics(),
      static_cast<float>(dynConfig_.pnp_inlier_max_rpj_error),
      (calibrationItrCnt_ > 1),
      newExtrinsics);

    // message publishing calibration result
    CalibrationResultMsg calibResultMsg;

    //--- check if calibration is to be kept
    if (((dynConfig_.pnp_limit_board_rpj_error &&
          pnpRetVal.first <= dynConfig_.pnp_board_max_rpj_error) ||
         (dynConfig_.pnp_limit_board_rpj_error == false)) &&
        pnpRetVal.second >= dynConfig_.pnp_board_min_inliers)
    {
        ROS_INFO("[%s]"
                 "\n\t> Calibration accepted!"
                 "\n\t> Mean Reprojection Error: %f px"
                 "\n\t> Inliers: %i pnts",
                 nodeletName_.c_str(), pnpRetVal.first, pnpRetVal.second);

        //--- store into member variable, and configure and apply frustum culling
        sensorExtrinsics_.push_back(newExtrinsics);
        configureAndApplyFrustumCulling();

        //--- store into message
        calibResultMsg.isSuccessful = true;

        //--- increment iteration count
        calibrationItrCnt_++;
    }
    else
    {
        ROS_WARN("[%s]"
                 "\n\t> Calibration rejected!"
                 "\n\t> Mean Reprojection Error: %f px (max. threshold: %f px)"
                 "\n\t> Inliers: %i pnts (min. threshold: %i pnts)"
                 "\n\t> Removing latest observations.",
                 nodeletName_.c_str(), pnpRetVal.first, dynConfig_.pnp_board_max_rpj_error,
                 pnpRetVal.second, dynConfig_.pnp_board_min_inliers);

        //--- remove all observations from this calibration iteration
        pCamDataProcessor_->removeCalibIteration(calibrationItrCnt_);
        pLidarDataProcessor_->removeCalibIteration(calibrationItrCnt_);

        //--- store into message
        calibResultMsg.isSuccessful = false;
    }

    calibResultPub_.publish(calibResultMsg);
}

//==================================================================================================
void ExtrinsicCameraLidarCalibrationNodelet::configureAndApplyFrustumCulling()
{
    //--- construct frustum culling pose
    Eigen::Matrix4f RTmatrix;
    cv::cv2eigen(cv::Mat(sensorExtrinsics_.back().getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF)),
                 RTmatrix);
    Eigen::Matrix4f cam2robot;
    cam2robot << 0, 0, 1, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    Eigen::Matrix4f frustumCullingCamPose = RTmatrix * cam2robot;

    //--- configure filter
    pcl::FrustumCulling<InputPointType>::Ptr pFilter =
      pcl::FrustumCulling<InputPointType>::Ptr(new pcl::FrustumCulling<InputPointType>());
    pFilter->setHorizontalFOV(
      static_cast<float>(pCamDataProcessor_->getCameraIntrinsics().getHFov()) + 10.f);
    pFilter->setVerticalFOV(
      static_cast<float>(pCamDataProcessor_->getCameraIntrinsics().getVFov()) + 10.f);
    pFilter->setCameraPose(frustumCullingCamPose);
    pFilter->setNearPlaneDistance(0.01f);
    pFilter->setFarPlaneDistance(10.f);

    //--- push back filter into list
    pFrustumCullingFilters_.push_back(pFilter);

    //--- set filter
    if (pLidarDataProcessor_)
    {
        pLidarDataProcessor_->setPreprocFilter(pFilter);
    }
}

//==================================================================================================
void ExtrinsicCameraLidarCalibrationNodelet::dynReconfigureCallback(
  ExtrinsicCameraLidarCalibrationConfig& iConfig, uint32_t level)
{
    UNUSED_VAR(level);

    //--- store object into member
    dynConfig_ = iConfig;

    //--- set to lidar processor
    LidarDataProcessor::setDynConfigParamsToProcessor(dynConfig_, pLidarDataProcessor_.get());
}

//==================================================================================================
void ExtrinsicCameraLidarCalibrationNodelet::finalizeCalibration()
{
    if (!pCamDataProcessor_->isCameraIntrinsicsSet())
    {
        ROS_ERROR("[%s] Could not finalize calibration. Camera intrinsics are not set",
                  nodeletName_.c_str());
        return;
    }

    //--- get all observations from data processors
    std::set<uint> cameraObservationIds;
    std::vector<cv::Point2f> cameraCornerObservations;
    pCamDataProcessor_->getOrderedObservations(cameraObservationIds, cameraCornerObservations);

    std::set<uint> lidarObservationIds;
    std::vector<cv::Point3f> lidarCornerObservations;
    pLidarDataProcessor_->getOrderedObservations(lidarObservationIds, lidarCornerObservations);

    //--- remove observations that do not have a correspondence in the other list
    removeCornerObservationsWithoutCorrespondence(cameraObservationIds,
                                                  lidarObservationIds,
                                                  lidarCornerObservations);
    removeCornerObservationsWithoutCorrespondence(lidarObservationIds,
                                                  cameraObservationIds,
                                                  cameraCornerObservations);

    if (cameraObservationIds.empty() || lidarObservationIds.empty())
    {
        ROS_ERROR("[%s] Could not finalize calibration. No common observations available.",
                  nodeletName_.c_str());
        return;
    }

    //--- do pnp calibration using all observed poses of calibration target
    lib3d::Extrinsics finalSensorExtrinsics; //  sensor extrinsics after pnp calibration using all targets
    std::pair<double, int> pnpRetVal = runPnp(
      cameraCornerObservations.cbegin(),
      cameraCornerObservations.cend(),
      lidarCornerObservations.cbegin(),
      lidarCornerObservations.cend(),
      pCamDataProcessor_->cameraIntrinsics(),
      static_cast<float>(dynConfig_.pnp_inlier_max_rpj_error),
      false,
      finalSensorExtrinsics);

    std::pair<tf::Vector3, tf::Vector3> xyz_rpy_stdDev;

    //--- set calibration meta data
    calibResult_.calibrations[0].srcSensorName = srcSensorName_;
    calibResult_.calibrations[0].srcFrameId    = srcFrameId_;
    calibResult_.calibrations[0].refSensorName = refSensorName_;
    calibResult_.calibrations[0].refFrameId    = refFrameId_;
    calibResult_.calibrations[0].baseFrameId   = baseFrameId_;

    //--- get transformation from lib3d::Extrinsics.
    // resulting transformation from ref to src sensor
    tf::Transform refToSrcTransform;
    utils::setTfTransformFromCameraExtrinsics(finalSensorExtrinsics,
                                              refToSrcTransform);
    calibResult_.calibrations[0].XYZ = refToSrcTransform.inverse().getOrigin(); // invert to get LOCAL_2_REF
    double roll, pitch, yaw;
    refToSrcTransform.inverse().getBasis().getRPY(roll, pitch, yaw); // invert to get LOCAL_2_REF
    calibResult_.calibrations[0].RPY = tf::Vector3(roll, pitch, yaw);

    //--- store reprojection error
    calibResult_.error = std::make_pair("Mean Reprojection Error (in pixel)", pnpRetVal.first);

    //--- computer target pose deviation if more than 1 observation is available
    if (pSrcDataProcessor_->getNumCalibIterations() > 1 &&
        pRefDataProcessor_->getNumCalibIterations() > 1)
    {
        calibResult_.target_poses_stdDev =
          computeTargetPoseStdDev(pSrcDataProcessor_->getCalibrationTargetPoses(),
                                  pRefDataProcessor_->getCalibrationTargetPoses());
    }

    //--- store meta information into calibResult
    calibResult_.numObservations = static_cast<int>(pLidarDataProcessor_->getNumCalibIterations());

    //--- calculate additional sensor calibrations if camera is to be calibrated as stereo camera
    if (isStereoCamera_ && rightCameraInfo_.width != 0)
        calculateAdditionalStereoCalibrations();
    else if (isStereoCamera_ && rightCameraInfo_.width == 0)
        ROS_ERROR("[%s] Could not calculate additional sensor calibrations as 'camera info' data "
                  "of right camera is not available.",
                  nodeletName_.c_str());

    //--- print out final transformation
    ROS_INFO("[%s]"
             "\n==================================================================================="
             "\n%s"
             "\n===================================================================================",
             nodeletName_.c_str(), calibResult_.toString().c_str());

    // message publishing calibration result
    CalibrationResultMsg calibResultMsg;
    calibResultPub_.publish(calibResultMsg);
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibrationNodelet::initializeDataProcessors()
{

    bool isSuccessful = true;

    //--- initialize camera data processor
    pCamDataProcessor_.reset(
      new CameraDataProcessor(nodeletName_, cameraSensorName_, calibTargetFilePath_));
    if (pCamDataProcessor_)
    {
        pCamDataProcessor_->setImageState(imageState_);
        pCamDataProcessor_->initializePublishers(pnh_);
    }
    else
        isSuccessful = false;

    //--- initialize lidar data processor
    pLidarDataProcessor_.reset(new LidarDataProcessor(nodeletName_, lidarSensorName_,
                                                      calibTargetFilePath_));
    if (pLidarDataProcessor_)
    {
        pLidarDataProcessor_->initializePublishers(pnh_);
        LidarDataProcessor::setDynConfigParamsToProcessor(dynConfig_, pLidarDataProcessor_.get());
    }
    else
    {
        isSuccessful = false;
    }

    return isSuccessful;
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibrationNodelet::initializeServices(ros::NodeHandle& ioNh)
{
    bool isSuccessful = ExtrinsicCalibrationBase::initializeServices(ioNh);

    //--- service to get camera intrinsics
    cameraIntrSrv_ = ioNh.advertiseService(
      REQUEST_CAM_INTRINSICS_SRV_NAME,
      &ExtrinsicCameraLidarCalibrationNodelet::onRequestCameraIntrinsics, this);

    return isSuccessful;
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibrationNodelet::initializeSubscribers(ros::NodeHandle& ioNh)
{
    //--- subscribe to topics
    image_transport::ImageTransport imgTransp(ioNh);
    imageSubsc_.subscribe(imgTransp, cameraImageTopic_, 1);
    cloudSubsc_.subscribe(ioNh, lidarCloudTopic_, 1);

    //--- initialize synchronizers
    if (useExactSync_)
    {
        pImgCloudExactSync_.reset(new message_filters::Synchronizer<ImgCloudExactSync>(
          ImgCloudExactSync(10), imageSubsc_, cloudSubsc_));
        pImgCloudExactSync_->registerCallback(
          boost::bind(&ExtrinsicCameraLidarCalibrationNodelet::onSensorDataReceived, this, _1, _2));
        pImgCloudExactSync_->setReset(true);
    }
    else
    {
        pImgCloudApproxSync_.reset(new message_filters::Synchronizer<ImgCloudApproxSync>(
          ImgCloudApproxSync(syncQueueSize_), imageSubsc_, cloudSubsc_));
        pImgCloudApproxSync_->registerCallback(
          boost::bind(&ExtrinsicCameraLidarCalibrationNodelet::onSensorDataReceived, this, _1, _2));
        pImgCloudApproxSync_->setReset(true);
    }

    return true;
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibrationNodelet::initializeWorkspaceObjects()
{
    bool retVal = true;

    retVal &= CalibrationBase::initializeWorkspaceObjects();

    //--- initialize calibration workspace
    fs::path calibWsPath = robotWsPath_;
    calibWsPath /=
      std::string(cameraSensorName_ + "_" + lidarSensorName_ + "_extrinsic_calibration");
    pCalibrationWs_ =
      std::make_shared<ExtrinsicCameraLidarCalibWorkspace>(calibWsPath, nodeletName_);
    retVal &= (pCalibrationWs_ != nullptr);

    if (pCalibrationWs_)
    {
        //--- create backup
        retVal &= pCalibrationWs_->createBackup();
    }

    return retVal;
}

//==================================================================================================
void ExtrinsicCameraLidarCalibrationNodelet::onInit()
{
    //--- get global and private node handle
    nh_  = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    //--- set nodelet name
    nodeletName_ = Nodelet::getName();

    //--- read launch parameters
    isInitialized_ = readLaunchParameters(pnh_);

    //--- if reading of launch parameters has returned with false, i.e. if error occurred, return.
    if (isInitialized_ == false)
        return;

    //--- initialize services
    isInitialized_ &= initializeServices(pnh_);

    //--- initialize workspace objects
    isInitialized_ &= initializeWorkspaceObjects();

    //--- create and start calibration workflow;
    isInitialized_ &= CalibrationBase::createAndStartCalibrationWorkflow(nh_);
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibrationNodelet::onRequestCameraIntrinsics(
  CameraIntrinsics::Request& iReq,
  CameraIntrinsics::Response& oRes)
{
    UNUSED_VAR(iReq);

    lib3d::Intrinsics cameraIntr;
    if (!isInitialized_ || pCamDataProcessor_ == nullptr)
        cameraIntr = lib3d::Intrinsics();
    else
        cameraIntr = pCamDataProcessor_->getCameraIntrinsics();

    //--- image size
    oRes.intrinsics.width  = cameraIntr.getWidth();
    oRes.intrinsics.height = cameraIntr.getHeight();

    //--- K
    cv::Mat K = cv::Mat(cameraIntr.getK_as3x3());
    std::memcpy(oRes.intrinsics.K.data(), K.data, 9 * K.elemSize1());

    //--- distortion
    int nDistParams   = cameraIntr.getDistortionCoeffs().total();
    oRes.intrinsics.D = std::vector<double>(nDistParams);
    std::memcpy(oRes.intrinsics.D.data(),
                cameraIntr.getDistortionCoeffs().data,
                nDistParams * cameraIntr.getDistortionCoeffs().elemSize1());

    //--- P
    cv::Mat P = cv::Mat(cameraIntr.getK_as3x4());
    std::memcpy(oRes.intrinsics.P.data(), P.data, 12 * K.elemSize1());

    //--- image state
    oRes.image_state = IMG_STATE_2_STR.find(imageState_)->second;

    return true;
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibrationNodelet::onRequestRemoveObservation(
  multisensor_calibration::RemoveLastObservation::Request& iReq,
  multisensor_calibration::RemoveLastObservation::Response& oRes)
{
    UNUSED_VAR(iReq);

    //--- if there is a calibration to be removed, remove all observations from this iteration
    if (calibrationItrCnt_ > 1)
    {

        //--- get ownership of mutex
        std::lock_guard<std::mutex> guard(dataProcessingMutex_);

        calibrationItrCnt_--;

        pCamDataProcessor_->removeCalibIteration(calibrationItrCnt_);
        pLidarDataProcessor_->removeCalibIteration(calibrationItrCnt_);

        //--- pop last sensor extrinsic
        sensorExtrinsics_.pop_back();

        //--- pop last frustum culling filter from stack and set previous filter
        pFrustumCullingFilters_.pop_back();
        if (pFrustumCullingFilters_.size() > 0 && pLidarDataProcessor_)
            pLidarDataProcessor_->setPreprocFilter(pFrustumCullingFilters_.back());
        else if (pFrustumCullingFilters_.size() == 0 && pLidarDataProcessor_)
            pLidarDataProcessor_->setPreprocFilter(nullptr);

        oRes.isAccepted = true;
        oRes.msg        = "Last observation successfully removed!"
                          "\n\t> Remaining number of observations: " +
                   std::to_string(pCamDataProcessor_->getNumCalibIterations());
    }
    else
    {
        oRes.isAccepted = false;
        oRes.msg        = "No observation available to be removed!";
    }

    ROS_INFO("[%s] %s", nodeletName_.c_str(), oRes.msg.c_str());

    return true;
}

//==================================================================================================
void ExtrinsicCameraLidarCalibrationNodelet::onSensorDataReceived(
  const InputImage_Message_T::ConstPtr& ipImgMsg,
  const InputCloud_Message_T::ConstPtr& ipCloudMsg)
{
    //--- check if nodelet is initialized
    if (!isInitialized_)
    {
        ROS_ERROR("[%s]"
                  "\n\t> Nodelet is not initialized.",
                  nodeletName_.c_str());
        return;
    }
    if (pCamDataProcessor_ == nullptr)
    {
        ROS_ERROR("[%s]"
                  "\n\t> Camera data processor is not initialized.",
                  nodeletName_.c_str());
        return;
    }
    if (pLidarDataProcessor_ == nullptr)
    {
        ROS_ERROR("[%s]"
                  "\n\t> Lidar data processor is not initialized.",
                  nodeletName_.c_str());
        return;
    }

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(dataProcessingMutex_);

    //--- convert input messages to data
    bool isConversionSuccessful = true;

    // camera image
    cv::Mat cameraImage;
    isConversionSuccessful &= pCamDataProcessor_->getSensorDataFromMsg(ipImgMsg, cameraImage);

    // point cloud
    pcl::PointCloud<InputPointType> pointCloud;
    isConversionSuccessful &= pLidarDataProcessor_->getSensorDataFromMsg(ipCloudMsg, pointCloud);

    if (!isConversionSuccessful)
    {
        ROS_ERROR("[%s] Something went wrong in getting the sensor data from the input messages.",
                  nodeletName_.c_str());
        return;
    }

    //--- camera intrinsics is not set to camera data processor,
    //--- wait for camera_info message and set intrinsics
    if (!pCamDataProcessor_->isCameraIntrinsicsSet())
    {
        bool isSuccessful = initializeCameraIntrinsics(pCamDataProcessor_.get());
        if (!isSuccessful)
            return;
    }

    //--- set frame ids and initialize sensor extrinsics if applicable
    if (imageFrameId_ != ipImgMsg->header.frame_id ||
        cloudFrameId_ != ipCloudMsg->header.frame_id)
    {
        imageFrameId_ = ipImgMsg->header.frame_id;
        cloudFrameId_ = ipCloudMsg->header.frame_id;

        //--- if base frame id is not empty and unequal to refCloudFrameId use baseFrameID as
        //--- reference frame id.
        std::string tmpRefFrameId = cloudFrameId_;
        if (!baseFrameId_.empty() && baseFrameId_ != cloudFrameId_)
        {
            tmpRefFrameId = baseFrameId_;

            if (tfListener_.frameExists(baseFrameId_))
            {
                try
                {
                    tf::StampedTransform transform;
                    tfListener_.lookupTransform(baseFrameId_, cloudFrameId_,
                                                ros::Time(0), transform);
                    pLidarDataProcessor_->setDataTransform(
                      std::make_shared<tf::Transform>(transform));
                }
                catch (tf::TransformException& ex)
                {
                    ROS_ERROR("[%s]"
                              "\n\t> tf::TransformException: %s",
                              nodeletName_.c_str(), ex.what());
                }
            }
            else
            {
                ROS_WARN("[%s]"
                         "\n\t> Base Frame %s does not exists! "
                         "Removing base frame and calibrating relative to reference cloud.",
                         nodeletName_.c_str(), baseFrameId_.c_str());
                baseFrameId_ = "";
                pLidarDataProcessor_->setDataTransform(nullptr);
            }
        }

        //--- set sensor extrinsics from either cloud or base frame id and apply frustum culling
        if (useTfTreeAsInitialGuess_ &&
            setSensorExtrinsicsFromFrameIds(imageFrameId_, tmpRefFrameId))
        {
            //--- update preprocessing filter with current sensorExtrinsics_
            configureAndApplyFrustumCulling();
        }
        else
        {
            //--- reset preprocessing filter
            pLidarDataProcessor_->setPreprocFilter(nullptr);
        }
    }

    // Level at which to do the processing
    CameraDataProcessor::EProcessingLevel procLevel = (captureCalibrationTarget_)
                                                        ? CameraDataProcessor::TARGET_DETECTION
                                                        : CameraDataProcessor::PREVIEW;

    //--- process camera data asynchronously
    std::future<CameraDataProcessor::EProcessingResult> camProcFuture =
      std::async(&CameraDataProcessor::processData,
                 pCamDataProcessor_,
                 cameraImage,
                 procLevel);

    //--- process lidar data asynchronously
    std::future<LidarDataProcessor::EProcessingResult> lidarProcFuture =
      std::async(&LidarDataProcessor::processData,
                 pLidarDataProcessor_,
                 pointCloud,
                 static_cast<LidarDataProcessor::EProcessingLevel>(procLevel));

    //--- wait for processing to return
    CameraDataProcessor::EProcessingResult camProcResult  = camProcFuture.get();
    LidarDataProcessor::EProcessingResult lidarProcResult = lidarProcFuture.get();

    //--- if processing level is set to preview, publish preview from each sensor if successful
    if (procLevel == CameraDataProcessor::PREVIEW)
    {
        if (camProcResult == CameraDataProcessor::SUCCESS)
            pCamDataProcessor_->publishPreview(ipImgMsg->header);
        if (lidarProcResult == LidarDataProcessor::SUCCESS)
            pLidarDataProcessor_->publishPreview(ipCloudMsg->header.stamp,
                                                 (baseFrameId_.empty())
                                                   ? cloudFrameId_
                                                   : baseFrameId_);
    }
    //--- else if, processing level is target_detection,
    //--- calibrate only if processing for both sensors is successful
    else if (procLevel == CameraDataProcessor::TARGET_DETECTION)
    {
        if (camProcResult == CameraDataProcessor::SUCCESS &&
            lidarProcResult == LidarDataProcessor::SUCCESS)
        {
            //--- publish detections
            pCamDataProcessor_->publishLastTargetDetection(ipImgMsg->header);
            pLidarDataProcessor_->publishLastTargetDetection(ipCloudMsg->header.stamp,
                                                             (baseFrameId_.empty())
                                                               ? cloudFrameId_
                                                               : baseFrameId_);

            //--- do calibration
            calibrateLastObservation();
        }
        else
        {
            if (camProcResult != CameraDataProcessor::SUCCESS &&
                lidarProcResult == LidarDataProcessor::SUCCESS)
                pLidarDataProcessor_->removeCalibIteration(calibrationItrCnt_);

            if (camProcResult == CameraDataProcessor::SUCCESS &&
                lidarProcResult != LidarDataProcessor::SUCCESS)
                pCamDataProcessor_->removeCalibIteration(calibrationItrCnt_);

            CalibrationResultMsg calibResultMsg;
            calibResultMsg.isSuccessful = false;
            calibResultPub_.publish(calibResultMsg);
        }

        //--- if data processor is not pending for more data, set capturing flag to false
        if (camProcResult != CameraDataProcessor::PENDING &&
            lidarProcResult != LidarDataProcessor::PENDING)
        {
            captureCalibrationTarget_ = false;
        }
    }
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibrationNodelet::saveCalibrationSettingsToWorkspace()
{
    if (!Extrinsic2d3dCalibrationBase::saveCalibrationSettingsToWorkspace())
        return false;

    QSettings* pCalibSettings = pCalibrationWs_->settingsPtr();
    if (!pCalibSettings)
        return false;

    //--- lidar sensor name
    pCalibSettings->setValue("lidar/sensor_name",
                             QString::fromStdString(lidarSensorName_));

    //--- camera image topic
    pCalibSettings->setValue("lidar/cloud_topic",
                             QString::fromStdString(lidarCloudTopic_));

    //--- sync queue
    pCalibSettings->setValue("misc/sync_queue_size",
                             QVariant::fromValue(syncQueueSize_));

    //--- exact sync
    pCalibSettings->setValue("misc/use_exact_sync",
                             QVariant::fromValue(useExactSync_));

    //--- sync settings file
    pCalibSettings->sync();

    return true;
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibrationNodelet::readLaunchParameters(const ros::NodeHandle& iNh)
{
    if (!Extrinsic2d3dCalibrationBase::readLaunchParameters(iNh))
        return false;

    //--- lidar_sensor_name
    lidarSensorName_ =
      readStringLaunchParameter(iNh, "lidar_sensor_name", DEFAULT_LIDAR_SENSOR_NAME);

    //--- lidar_cloud_topic
    lidarCloudTopic_ =
      readStringLaunchParameter(iNh, "lidar_cloud_topic", DEFAULT_LIDAR_CLOUD_TOPIC);

    //--- sync queue
    syncQueueSize_ = readNumericLaunchParameter<int>(iNh, "sync_queue_size",
                                                     DEFAULT_SYNC_QUEUE_SIZE, 1, INT_MAX);

    //--- exact sync
    iNh.param<bool>("use_exact_sync", useExactSync_, false);

    return true;
}

//==================================================================================================
void ExtrinsicCameraLidarCalibrationNodelet::reset()
{
    ExtrinsicCalibrationBase::reset();

    pCamDataProcessor_->reset();
    pLidarDataProcessor_->reset();
    pLidarDataProcessor_->setPreprocFilter(nullptr);
}

//==================================================================================================
bool ExtrinsicCameraLidarCalibrationNodelet::shutdownSubscribers()
{
    //--- check if node is initialized
    if (!isInitialized_)
        return false;

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(dataProcessingMutex_);

    //--- unsubscribe subscribers
    imageSubsc_.unsubscribe();
    cloudSubsc_.unsubscribe();

    return true;
}

} // namespace multisensor_calibration

// Export the class as a plugin using the PLUGINLIB_EXPORT_CLASS macro.
PLUGINLIB_EXPORT_CLASS(multisensor_calibration::ExtrinsicCameraLidarCalibrationNodelet, nodelet::Nodelet)
