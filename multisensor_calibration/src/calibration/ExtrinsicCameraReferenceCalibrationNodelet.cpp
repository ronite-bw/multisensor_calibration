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

#include "../include/multisensor_calibration/calibration/ExtrinsicCameraReferenceCalibrationNodelet.h"

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
ExtrinsicCameraReferenceCalibrationNodelet::ExtrinsicCameraReferenceCalibrationNodelet() :
  Extrinsic2d3dCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>(
    EXTRINSIC_CAMERA_REFERENCE_CALIBRATION)
{
    //--- connect dyn reconfigure callback
    dynConfigServer_.setCallback(
      boost::bind(&ExtrinsicCameraReferenceCalibrationNodelet::dynReconfigureCallback,
                  this, _1, _2));
}

//==================================================================================================
ExtrinsicCameraReferenceCalibrationNodelet::~ExtrinsicCameraReferenceCalibrationNodelet()
{
    //--- reset pointers message filters before sensor processors in order to avoid seg fault during
    //--- disconnection of callbacks.
    pCamDataProcessor_.reset();
    pRefDataProcessor_.reset();
}

//==================================================================================================
void ExtrinsicCameraReferenceCalibrationNodelet::dynReconfigureCallback(
  ExtrinsicCameraReferenceCalibrationConfig& iConfig, uint32_t level)
{
    UNUSED_VAR(level);

    //--- store object into member
    dynConfig_ = iConfig;
}

//==================================================================================================
void ExtrinsicCameraReferenceCalibrationNodelet::finalizeCalibration()
{
    if (!pCamDataProcessor_->isCameraIntrinsicsSet())
    {
        ROS_ERROR("[%s] Could not finalize calibration. Camera intrinsics are not set.",
                  nodeletName_.c_str());
        return;
    }

    //--- get all observations from data processors
    std::set<uint> cameraObservationIds;
    std::vector<cv::Point2f> cameraCornerObservations;
    pCamDataProcessor_->getOrderedObservations(cameraObservationIds, cameraCornerObservations);

    std::set<uint> refObservationIds;
    std::vector<cv::Point3f> refCornerObservations;
    pRefDataProcessor_->getOrderedObservations(refObservationIds, refCornerObservations);

    //--- remove observations that do not have a correspondence in the other list
    removeCornerObservationsWithoutCorrespondence(cameraObservationIds,
                                                  refObservationIds,
                                                  refCornerObservations);
    removeCornerObservationsWithoutCorrespondence(refObservationIds,
                                                  cameraObservationIds,
                                                  cameraCornerObservations);

    if (cameraObservationIds.empty() || refObservationIds.empty())
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
      refCornerObservations.cbegin(),
      refCornerObservations.cend(),
      pCamDataProcessor_->cameraIntrinsics(),
      static_cast<float>(dynConfig_.pnp_inlier_max_rpj_error),
      false,
      finalSensorExtrinsics);

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
    calibResult_.numObservations = static_cast<int>(pRefDataProcessor_->getNumCalibIterations());

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
    calibResultMsg.isSuccessful = true;
    calibResultPub_.publish(calibResultMsg);
}

//==================================================================================================
bool ExtrinsicCameraReferenceCalibrationNodelet::initializeDataProcessors()
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

    //--- initialize reference data processor
    pRefDataProcessor_.reset(
      new ReferenceDataProcessor3d(nodeletName_, referenceName_, calibTargetFilePath_));
    if (pRefDataProcessor_)
    {
        pRefDataProcessor_->initializeServices(pnh_);
        pRefDataProcessor_->initializePublishers(pnh_);
    }
    else
    {
        isSuccessful = false;
    }

    return isSuccessful;
}

//==================================================================================================
bool ExtrinsicCameraReferenceCalibrationNodelet::initializeServices(ros::NodeHandle& ioNh)
{
    bool isSuccessful = ExtrinsicCalibrationBase::initializeServices(ioNh);

    //--- service to get camera intrinsics
    cameraIntrSrv_ = ioNh.advertiseService(
      REQUEST_CAM_INTRINSICS_SRV_NAME,
      &ExtrinsicCameraReferenceCalibrationNodelet::onRequestCameraIntrinsics, this);

    return isSuccessful;
}

//==================================================================================================
bool ExtrinsicCameraReferenceCalibrationNodelet::initializeSubscribers(ros::NodeHandle& ioNh)
{
    //--- subscribe to image topics
    image_transport::ImageTransport imgTransp(ioNh);
    imageSubsc_ = imgTransp.subscribe(
      cameraImageTopic_, 1,
      &ExtrinsicCameraReferenceCalibrationNodelet::onSensorDataReceived, this);

    return true;
}

//==================================================================================================
bool ExtrinsicCameraReferenceCalibrationNodelet::initializeWorkspaceObjects()
{
    bool retVal = true;

    retVal &= CalibrationBase::initializeWorkspaceObjects();

    //--- initialize calibration workspace
    fs::path calibWsPath = robotWsPath_;
    calibWsPath /=
      std::string(cameraSensorName_ + "_" + refSensorName_ + "_extrinsic_calibration");
    pCalibrationWs_ =
      std::make_shared<ExtrinsicCameraReferenceCalibWorkspace>(calibWsPath, nodeletName_);
    retVal &= (pCalibrationWs_ != nullptr);

    if (pCalibrationWs_)
    {
        //--- create backup
        retVal &= pCalibrationWs_->createBackup();
    }

    return retVal;
}

//==================================================================================================
void ExtrinsicCameraReferenceCalibrationNodelet::onInit()
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
bool ExtrinsicCameraReferenceCalibrationNodelet::onRequestCameraIntrinsics(
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
bool ExtrinsicCameraReferenceCalibrationNodelet::onRequestRemoveObservation(
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
        pRefDataProcessor_->removeCalibIteration(calibrationItrCnt_);

        //--- since no per iteration calibration is performed no sensorExtrinsic needs
        //-- to be removed
        // sensorExtrinsics_.pop_back()

        oRes.isAccepted = true;
        oRes.msg        = "Last observation successfully removed!"
                          "\n\t> Remaining number of observations: " +
                   std::to_string(pCamDataProcessor_->getNumCalibIterations()) + " (src), " +
                   std::to_string(pRefDataProcessor_->getNumCalibIterations()) + " (ref).";
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
void ExtrinsicCameraReferenceCalibrationNodelet::onSensorDataReceived(
  const InputImage_Message_T::ConstPtr& ipImgMsg)
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
    if (pRefDataProcessor_ == nullptr)
    {
        ROS_ERROR("[%s]"
                  "\n\t> Reference data processor is not initialized.",
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
    if (imageFrameId_ != ipImgMsg->header.frame_id)
    {
        imageFrameId_ = ipImgMsg->header.frame_id;

        //--- compute sensor extrinsics between source frame id and ref or base frame id
        //--- if base frame id is not empty and unequal to refCloudFrameId also get transform
        //--- between refFrameID and baseFrameID and pass to refLidarProcessor.
        if (!baseFrameId_.empty() && baseFrameId_ != refFrameId_)
        {
            if (useTfTreeAsInitialGuess_)
                setSensorExtrinsicsFromFrameIds(imageFrameId_, baseFrameId_);

            if (tfListener_.frameExists(baseFrameId_))
            {
                try
                {
                    tf::StampedTransform transform;
                    tfListener_.lookupTransform(baseFrameId_, refFrameId_,
                                                ros::Time(0), transform);
                    pRefDataProcessor_->setDataTransform(
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
                pRefDataProcessor_->setDataTransform(nullptr);
            }
        }
        else
        {
            if (useTfTreeAsInitialGuess_)
                setSensorExtrinsicsFromFrameIds(imageFrameId_, refFrameId_);
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

    //--- wait for processing to return
    CameraDataProcessor::EProcessingResult camProcResult = camProcFuture.get();

    //--- if processing level is set to preview, publish preview from each sensor if successful
    if (procLevel == CameraDataProcessor::PREVIEW)
    {
        if (camProcResult == CameraDataProcessor::SUCCESS)
            pCamDataProcessor_->publishPreview(ipImgMsg->header);
    }
    //--- else if, processing level is target_detection,
    //--- calibrate only if processing for both sensors is successful
    else if (procLevel == CameraDataProcessor::TARGET_DETECTION)
    {
        CalibrationResultMsg calibResultMsg;
        if (camProcResult == CameraDataProcessor::SUCCESS)
        {
            //--- publish detections
            pCamDataProcessor_->publishLastTargetDetection(ipImgMsg->header);

            //--- Other extrinsic calibration routines, i.e. camera-lidar or lidar-lidar, run
            //--- an intermediate calibration at this point. This will increase the calibration
            //--- iteration counter (calibrationItrCnt_) which, in turn, is evaluated to be larger
            //--- than 1 before finalizing the calibration. The calibration iteration counter
            //--- somehow also serves for an internal counter on how many common observations there
            //--- are. Since for the sensor-reference calibration, only a calibration is done at the
            //--- end, increase the counter when a observation in the source data was detected.
            calibrationItrCnt_++;

            calibResultMsg.isSuccessful = true;
        }
        else
        {
            calibResultMsg.isSuccessful = false;
        }

        calibResultPub_.publish(calibResultMsg);
    }

    //--- if data processor is not pending for more data, set capturing flag to false
    if (camProcResult != CameraDataProcessor::PENDING)
    {
        captureCalibrationTarget_ = false;
    }
}

//==================================================================================================
bool ExtrinsicCameraReferenceCalibrationNodelet::saveCalibrationSettingsToWorkspace()
{
    if (!Extrinsic2d3dCalibrationBase::saveCalibrationSettingsToWorkspace())
        return false;

    QSettings* pCalibSettings = pCalibrationWs_->settingsPtr();
    if (!pCalibSettings)
        return false;

    //--- reference name
    pCalibSettings->setValue("reference/name",
                             QString::fromStdString(referenceName_));

    //--- reference frame id
    pCalibSettings->setValue("reference/frame_id",
                             QString::fromStdString(refFrameId_));

    //--- sync settings file
    pCalibSettings->sync();

    return true;
}

//==================================================================================================
bool ExtrinsicCameraReferenceCalibrationNodelet::readLaunchParameters(const ros::NodeHandle& iNh)
{
    if (!Extrinsic2d3dCalibrationBase::readLaunchParameters(iNh))
        return false;

    //--- reference_name
    referenceName_ =
      readStringLaunchParameter(iNh, "reference_name", "reference");

    //--- ref_lidar_sensor_name
    refFrameId_ =
      readStringLaunchParameter(iNh, "reference_frame_id", "reference");

    //--- need to set ref in order for the calibration meta data to be complete. However,
    //--- this is not evaluated
    refTopicName_ = "/cloud";

    //--- initial guess is not supported for sensor-reference calibration
    useTfTreeAsInitialGuess_ = false;

    return true;
}

//==================================================================================================
void ExtrinsicCameraReferenceCalibrationNodelet::reset()
{
    ExtrinsicCalibrationBase::reset();

    pCamDataProcessor_->reset();
    pRefDataProcessor_->reset();
}

//==================================================================================================
bool ExtrinsicCameraReferenceCalibrationNodelet::shutdownSubscribers()
{
    //--- check if node is initialized
    if (!isInitialized_)
        return false;

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(dataProcessingMutex_);

    //--- unsubscribe subscribers
    imageSubsc_.shutdown();

    return true;
}

} // namespace multisensor_calibration

// Export the class as a plugin using the PLUGINLIB_EXPORT_CLASS macro.
PLUGINLIB_EXPORT_CLASS(multisensor_calibration::ExtrinsicCameraReferenceCalibrationNodelet, nodelet::Nodelet)
