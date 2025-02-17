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

#include "../include/multisensor_calibration/calibration/ExtrinsicLidarLidarCalibrationNodelet.h"

// Std
#include <fstream>
#include <functional>
#include <future>
#include <thread>

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

// ROS
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/CameraInfo.h>

// Qt
#include <QFile>

// small_gicp
#include <small_gicp/registration/registration_helper.hpp>

// multisensor_calibration
#include "../../include/multisensor_calibration/common/common.h"
#include "../../include/multisensor_calibration/common/utils.hpp"

namespace multisensor_calibration
{

//==================================================================================================
ExtrinsicLidarLidarCalibrationNodelet::ExtrinsicLidarLidarCalibrationNodelet() :
  Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>(EXTRINSIC_LIDAR_LIDAR_CALIBRATION),
  pCloudCloudApproxSync_(nullptr),
  pCloudCloudExactSync_(nullptr),
  alignGroundPlanes_(false),
  uprightFrameId_(""),
  syncQueueSize_(DEFAULT_SYNC_QUEUE_SIZE),
  useExactSync_(false)
{
    //--- connect dyn reconfigure callback
    dynConfigServer_.setCallback(
      boost::bind(&ExtrinsicLidarLidarCalibrationNodelet::dynReconfigureCallback, this, _1, _2));
}

//==================================================================================================
ExtrinsicLidarLidarCalibrationNodelet::~ExtrinsicLidarLidarCalibrationNodelet()
{
    //--- reset pointers message filters before sensor processors in order to avoid seg fault during
    //--- disconnection of callbacks.
    pCloudCloudApproxSync_.reset();
    pCloudCloudExactSync_.reset();
    pSrcLidarDataProcessor_.reset();
    pRefLidarDataProcessor_.reset();
}

//==================================================================================================
void ExtrinsicLidarLidarCalibrationNodelet::calibrateLastObservation()
{
    using namespace pcl::registration;

    //--- check that for this calibration iteration, both camera and lidar observations are available
    //--- i.e. if list is empty, or if last item in list is smaller than iterationCnt * 100, return
    if (pSrcLidarDataProcessor_->getNumCalibIterations() < calibrationItrCnt_ ||
        pRefLidarDataProcessor_->getNumCalibIterations() < calibrationItrCnt_)
        return;

    //--- copy marker corner observations to point cloud
    //--- make uneven in order to remove ambiguities
    pcl::PointCloud<pcl::PointXYZ>::Ptr pSrcMarkerCornerCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pRefMarkerCornerCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (uint i = 1; i <= calibrationItrCnt_; ++i)
    {
        //--- get observations from source LiDAR
        std::set<uint> srcObservationIds;
        std::vector<cv::Point3f> srcCornerObservations;
        pSrcLidarDataProcessor_->getOrderedObservations(srcObservationIds, srcCornerObservations,
                                                        i, 1);

        //--- get observations from reference LiDAR
        std::set<uint> refObservationIds;
        std::vector<cv::Point3f> refCornerObservations;
        pRefLidarDataProcessor_->getOrderedObservations(refObservationIds, refCornerObservations,
                                                        i, 1);

        //--- remove observations that do not have a correspondence in the other list
        removeCornerObservationsWithoutCorrespondence(srcObservationIds,
                                                      refObservationIds,
                                                      refCornerObservations);
        removeCornerObservationsWithoutCorrespondence(refObservationIds,
                                                      srcObservationIds,
                                                      srcCornerObservations);

        //--- if number of IDs is even (i.e. even number of markers), remove id and corners marker
        //--- with id of first correspondences to make orientation unique
        auto makeUneven = [&](std::set<uint>& ids, std::vector<cv::Point3f>& corners,
                              const uint& idToRemove)
        {
            if ((ids.size() % 2) == 0)
            {
                // find it to be removed in list
                std::set<uint>::iterator idItr = std::find(ids.begin(), ids.end(), idToRemove);
                if (idItr == ids.end())
                    return;

                //--- compute index if src iterator and remove id and observations
                uint idIdx               = std::distance(ids.begin(), idItr);
                auto cornersEaseStartPos = corners.begin() + (idIdx * 4);

                ids.erase(idItr);
                corners.erase(cornersEaseStartPos, cornersEaseStartPos + 4);
            }
        };
        uint idToRemove = *srcObservationIds.begin();
        makeUneven(srcObservationIds, srcCornerObservations, idToRemove);
        makeUneven(refObservationIds, refCornerObservations, idToRemove);

        //--- push remaining to point cloud
        auto pushToCloud = [&](const std::vector<cv::Point3f>& cornerVec,
                               pcl::PointCloud<pcl::PointXYZ>& cornerCloud)
        {
            for (auto itr = cornerVec.cbegin(); itr != cornerVec.cend(); ++itr)
            {
                cornerCloud.push_back(pcl::PointXYZ(itr->x, itr->y, itr->z));
            }
        };
        pushToCloud(srcCornerObservations, *pSrcMarkerCornerCloud);
        pushToCloud(refCornerObservations, *pRefMarkerCornerCloud);
    }

    // Correspondences of marker corners between src and ref cloud. Since the marker corners are
    // extracted in order, the correspondence list simply consists of increasing index numbers
    pcl::Correspondences correspondences;
    for (uint i = 0; i < pSrcMarkerCornerCloud->size(); ++i)
    {
        correspondences.push_back(pcl::Correspondence(i, i, 1.f));
    }

    //--- estimate sensor extrinsics from marker corners
    auto sensorExtrinsic = computeExtrinsicsFromPointCorrespondences<pcl::PointXYZ>(
      pSrcMarkerCornerCloud, pRefMarkerCornerCloud, correspondences);
    sensorExtrinsics_.push_back(sensorExtrinsic);

    //--- increment iteration count
    calibrationItrCnt_++;

    // message publishing calibration result
    CalibrationResultMsg calibResultMsg;
    calibResultMsg.isSuccessful = true;
    calibResultPub_.publish(calibResultMsg);
}

//==================================================================================================
void ExtrinsicLidarLidarCalibrationNodelet::dynReconfigureCallback(
  ExtrinsicLidarLidarCalibrationConfig& iConfig, uint32_t level)
{
    UNUSED_VAR(level);

    //--- store object into member
    dynConfig_ = iConfig;

    //--- set to lidar processors
    LidarDataProcessor::setDynConfigParamsToProcessor(dynConfig_, pSrcLidarDataProcessor_.get());
    LidarDataProcessor::setDynConfigParamsToProcessor(dynConfig_, pRefLidarDataProcessor_.get());
}

//==================================================================================================
void ExtrinsicLidarLidarCalibrationNodelet::finalizeCalibration()
{

    //--- merge target clouds

    // pointer to cloud holding calibration target clouds of all observations from the src lidar
    pcl::PointCloud<InputPointType>::Ptr pSrcLidarTargetClouds(
      new pcl::PointCloud<InputPointType>());

    // pointer to cloud holding calibration target clouds of all observations from the ref lidar
    pcl::PointCloud<InputPointType>::Ptr pRefLidarTargetClouds(
      new pcl::PointCloud<InputPointType>());

    auto srcLidarTargetCloudPtrs = pSrcLidarDataProcessor_->getCalibrationTargetCloudPtrs();
    for (pcl::PointCloud<InputPointType>::Ptr pSubCloud : srcLidarTargetCloudPtrs)
        pSrcLidarTargetClouds->insert(pSrcLidarTargetClouds->end(),
                                      pSubCloud->begin(),
                                      pSubCloud->end());
    auto refLidarTargetCloudPtrs = pRefLidarDataProcessor_->getCalibrationTargetCloudPtrs();
    for (pcl::PointCloud<InputPointType>::Ptr pSubCloud : refLidarTargetCloudPtrs)
        pRefLidarTargetClouds->insert(pRefLidarTargetClouds->end(),
                                      pSubCloud->begin(),
                                      pSubCloud->end());

    if (pSrcLidarTargetClouds->empty() || pRefLidarTargetClouds->empty())
    {
        ROS_ERROR("[%s] Could not finalize calibration. No common observations available.",
                  nodeletName_.c_str());
        return;
    }

    // list of indices of target points in source cloud
    pcl::IndicesPtr pSrcTargetIndices = nullptr;

    // list of indices of target points in reference cloud
    pcl::IndicesPtr pRefTargetIndices = nullptr;

    //--- additionally align ground planes if flag
    //--- whether an upright frame is specified, is already checked in 'readLaunchParameters'
    if (alignGroundPlanes_)
    {

        //--- fill list of point indices lying on targets with increasing number starting from 0
        //--- this is needed to only compute RMSE between target points and not to incorporate the
        //--- ground plane
        //--- to this end, only the points on the targets are in the corresponding point clouds, thus,
        //--- the size of the point clouds can be used as upper bound of the indices.
        pSrcTargetIndices.reset(new pcl::Indices(pSrcLidarTargetClouds->size()));
        std::iota(std::begin(*pSrcTargetIndices), std::end(*pSrcTargetIndices), 0);
        pRefTargetIndices.reset(new pcl::Indices(pRefLidarTargetClouds->size()));
        std::iota(std::begin(*pRefTargetIndices), std::end(*pRefTargetIndices), 0);

        //--- check if specified upright frame exists
        if (tfListener_.frameExists(uprightFrameId_))
        {
            tf::StampedTransform srcTransform, refTransform;

            bool isLookupSuccessful;
            try
            {
                tfListener_.lookupTransform(srcCloudFrameId_, uprightFrameId_,
                                            ros::Time(0), srcTransform);
                tfListener_.lookupTransform(((baseFrameId_.empty()) ? refCloudFrameId_ : baseFrameId_),
                                            uprightFrameId_,
                                            ros::Time(0), refTransform);

                isLookupSuccessful = true;
            }
            catch (tf::TransformException& ex)
            {
                ROS_ERROR("[%s] tf::TransformException in trying to get transform between "
                          "upright frame and sensor frames or base frame: %s"
                          "\nAlignment of ground planes will be skipped.",
                          nodeletName_.c_str(), ex.what());
                isLookupSuccessful = false;
            }

            //--- if lookup is successful, find ground planes and add to point clouds that are used
            //--- for alignment
            if (isLookupSuccessful)
            {
                //--- find upright vector in frame of sensor data by transforming the z-axis with
                //--- the rotational part of the transform between the upright frame and the
                //--- sensor frames.
                tf::Vector3 srcUprightVec = srcTransform.getBasis() * tf::Vector3(0.f, 0.f, 1.f);
                srcUprightVec.normalize();
                tf::Vector3 refUprightVec = refTransform.getBasis() * tf::Vector3(0.f, 0.f, 1.f);
                refUprightVec.normalize();

                // Pointer to ground plane cloud of reference data.
                pcl::PointCloud<InputPointType>::Ptr pRefGroundPlane(
                  new pcl::PointCloud<InputPointType>);

                // Pointer to ground plane cloud of source data.
                pcl::PointCloud<InputPointType>::Ptr pSrcGroundPlane(
                  new pcl::PointCloud<InputPointType>);

                //--- extract ground planes in sensor data with an angle tolerance of 5 degrees
                //--- to the upright vectors.
                LidarDataProcessor::extractPlaneFromPointCloud(
                  pRefLidarDataProcessor_->getLastInputCloud(), refUprightVec, 5, pRefGroundPlane);
                LidarDataProcessor::extractPlaneFromPointCloud(
                  pSrcLidarDataProcessor_->getLastInputCloud(), srcUprightVec, 5, pSrcGroundPlane);

                //--- insert the extracted ground planes into the point clouds used for the ICP
                pRefLidarTargetClouds->insert(pRefLidarTargetClouds->end(),
                                              pRefGroundPlane->begin(),
                                              pRefGroundPlane->end());

                pSrcLidarTargetClouds->insert(pSrcLidarTargetClouds->end(),
                                              pSrcGroundPlane->begin(),
                                              pSrcGroundPlane->end());
            }
        }
        else
        {
            ROS_ERROR("[%s] Specified upright frame with the id '%s' does not exist."
                      "\nAlignment of ground planes will be skipped.",
                      nodeletName_.c_str(), uprightFrameId_.c_str());
        }
    }

    //--- run ICP
    runIcp<InputPointType>(
      pSrcLidarTargetClouds, pRefLidarTargetClouds,
      static_cast<small_gicp::RegistrationSetting::RegistrationType>(dynConfig_.registration_icp_variant),
      dynConfig_.registration_icp_max_correspondence_distance,
      dynConfig_.registration_icp_rotation_tolerance,
      dynConfig_.registration_icp_translation_tolerance);

    //--- calculate RMSE
    //--- if no ground plane is used, the pointer to the indices will be nullptr, thus, the full
    //--- cloud will be used
    double rmse = utils::calculateRootMeanSquaredError<InputPointType, InputPointType>(
      pSrcLidarTargetClouds, pRefLidarTargetClouds,
      ExtrinsicCalibrationBase::sensorExtrinsics_.back(),
      pSrcTargetIndices, pRefTargetIndices);
    calibResult_.error = std::make_pair("Root Mean Squared Error (in m)", rmse);

    //--- computer target pose deviation if more than 1 observation is available
    if (pSrcDataProcessor_->getNumCalibIterations() > 1 &&
        pRefDataProcessor_->getNumCalibIterations() > 1)
    {
        calibResult_.target_poses_stdDev =
          computeTargetPoseStdDev(pSrcDataProcessor_->getCalibrationTargetPoses(),
                                  pRefDataProcessor_->getCalibrationTargetPoses());
    }

    //--- set calibration meta data
    calibResult_.calibrations[0].srcSensorName = srcSensorName_;
    calibResult_.calibrations[0].srcFrameId    = srcFrameId_;
    calibResult_.calibrations[0].refSensorName = refSensorName_;
    calibResult_.calibrations[0].refFrameId    = refFrameId_;
    calibResult_.calibrations[0].baseFrameId   = baseFrameId_;

    //--- get extrinsics
    const lib3d::Extrinsics& extrinsics = sensorExtrinsics_.back();

    //--- get transformation from lib3d::Extrinsics.
    // resulting transformation from ref to src sensor
    tf::Transform refToSrcTransform;
    utils::setTfTransformFromCameraExtrinsics(extrinsics,
                                              refToSrcTransform);
    calibResult_.calibrations[0].XYZ = refToSrcTransform.inverse().getOrigin(); // invert to get LOCAL_2_REF
    double roll, pitch, yaw;
    refToSrcTransform.inverse().getBasis().getRPY(roll, pitch, yaw); // invert to get LOCAL_2_REF
    calibResult_.calibrations[0].RPY = tf::Vector3(roll, pitch, yaw);

    //--- store meta information into calibResult
    calibResult_.numObservations = static_cast<int>(pRefDataProcessor_->getNumCalibIterations());

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
bool ExtrinsicLidarLidarCalibrationNodelet::initializeDataProcessors()
{

    bool isSuccessful = true;

    // Lambda function to initialize pointer to LidarDataProcessor
    auto initializeLidarDataProcessor = [&](std::shared_ptr<LidarDataProcessor>& iopProcessor,
                                            const std::string& iSensorName)
    {
        iopProcessor.reset(
          new LidarDataProcessor(nodeletName_, iSensorName, calibTargetFilePath_));
        if (iopProcessor)
        {
            iopProcessor->initializeServices(pnh_);
            iopProcessor->initializePublishers(pnh_);
            LidarDataProcessor::setDynConfigParamsToProcessor(dynConfig_, iopProcessor.get());
        }
        else
        {
            isSuccessful = false;
        }
    };

    //--- initialize data processors
    initializeLidarDataProcessor(pSrcLidarDataProcessor_, srcSensorName_);
    initializeLidarDataProcessor(pRefLidarDataProcessor_, refSensorName_);

    return isSuccessful;
}

//==================================================================================================
bool ExtrinsicLidarLidarCalibrationNodelet::initializeSubscribers(ros::NodeHandle& ioNh)
{
    //--- subscribe to topics
    srcCloudSubsc_.subscribe(ioNh, srcLidarCloudTopic_, 1);
    refCloudSubsc_.subscribe(ioNh, refLidarCloudTopic_, 1);

    //--- initialize synchronizers
    if (useExactSync_)
    {
        pCloudCloudExactSync_.reset(new message_filters::Synchronizer<CloudCloudExactSync>(
          CloudCloudExactSync(10), srcCloudSubsc_, refCloudSubsc_));
        pCloudCloudExactSync_->registerCallback(
          boost::bind(&ExtrinsicLidarLidarCalibrationNodelet::onSensorDataReceived, this, _1, _2));
        pCloudCloudExactSync_->setReset(true);
    }
    else
    {
        pCloudCloudApproxSync_.reset(new message_filters::Synchronizer<CloudCloudApproxSync>(
          CloudCloudApproxSync(syncQueueSize_), srcCloudSubsc_, refCloudSubsc_));
        pCloudCloudApproxSync_->registerCallback(
          boost::bind(&ExtrinsicLidarLidarCalibrationNodelet::onSensorDataReceived, this, _1, _2));
        pCloudCloudApproxSync_->setReset(true);
    }

    return true;
}

//==================================================================================================
bool ExtrinsicLidarLidarCalibrationNodelet::initializeWorkspaceObjects()
{
    bool retVal = true;

    retVal &= CalibrationBase::initializeWorkspaceObjects();

    //--- initialize calibration workspace
    fs::path calibWsPath = robotWsPath_;
    calibWsPath /=
      std::string(srcLidarSensorName_ + "_" + refLidarSensorName_ + "_extrinsic_calibration");
    pCalibrationWs_ =
      std::make_shared<ExtrinsicLidarLidarCalibWorkspace>(calibWsPath, nodeletName_);
    retVal &= (pCalibrationWs_ != nullptr);

    return retVal;
}

//==================================================================================================
void ExtrinsicLidarLidarCalibrationNodelet::onInit()
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
    isInitialized_ &= createAndStartCalibrationWorkflow(nh_);
}

//==================================================================================================
bool ExtrinsicLidarLidarCalibrationNodelet::onRequestRemoveObservation(
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

        pSrcDataProcessor_->removeCalibIteration(calibrationItrCnt_);
        pRefDataProcessor_->removeCalibIteration(calibrationItrCnt_);

        //--- pop last sensor extrinsic
        sensorExtrinsics_.pop_back();

        oRes.isAccepted = true;
        oRes.msg        = "Last observation successfully removed!"
                          "\n\t> Remaining number of observations: " +
                   std::to_string(pRefDataProcessor_->getNumCalibIterations());
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
void ExtrinsicLidarLidarCalibrationNodelet::onSensorDataReceived(
  const InputCloud_Message_T::ConstPtr& ipSrcCloudMsg,
  const InputCloud_Message_T::ConstPtr& ipRefCloudMsg)
{
    //--- check if nodelet is initialized
    if (!isInitialized_)
    {
        ROS_ERROR("[%s]"
                  "\n\t> Nodelet is not initialized.",
                  nodeletName_.c_str());
        return;
    }
    if (pSrcLidarDataProcessor_ == nullptr)
    {
        ROS_ERROR("[%s]"
                  "\n\t> Source lidar data processor is not initialized.",
                  nodeletName_.c_str());
        return;
    }
    if (pRefLidarDataProcessor_ == nullptr)
    {
        ROS_ERROR("[%s]"
                  "\n\t> Reference lidar data processor is not initialized.",
                  nodeletName_.c_str());
        return;
    }

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(dataProcessingMutex_);

    //--- convert input messages to data
    bool isConversionSuccessful = true;

    // source point cloud
    pcl::PointCloud<InputPointType> srcPointCloud;
    isConversionSuccessful &=
      pSrcLidarDataProcessor_->getSensorDataFromMsg(ipSrcCloudMsg, srcPointCloud);

    // reference point cloud
    pcl::PointCloud<InputPointType> refPointCloud;
    isConversionSuccessful &=
      pRefLidarDataProcessor_->getSensorDataFromMsg(ipRefCloudMsg, refPointCloud);

    if (!isConversionSuccessful)
    {
        ROS_ERROR("[%s] Something went wrong in getting the sensor data from the input messages.",
                  nodeletName_.c_str());
        return;
    }

    //--- set frame ids and initialize sensor extrinsics if applicable
    if (srcCloudFrameId_ != ipSrcCloudMsg->header.frame_id ||
        refCloudFrameId_ != ipRefCloudMsg->header.frame_id)
    {
        srcCloudFrameId_ = ipSrcCloudMsg->header.frame_id;
        refCloudFrameId_ = ipRefCloudMsg->header.frame_id;

        //--- compute sensor extrinsics between source frame id and ref or base frame id
        //--- if base frame id is not empty and unequal to refCloudFrameId also get transform
        //--- between refFrameID and baseFrameID and pass to refLidarProcessor.
        if (!baseFrameId_.empty() && baseFrameId_ != refCloudFrameId_)
        {
            if (useTfTreeAsInitialGuess_)
                setSensorExtrinsicsFromFrameIds(srcCloudFrameId_, baseFrameId_);

            if (tfListener_.frameExists(baseFrameId_))
            {
                try
                {
                    tf::StampedTransform transform;
                    tfListener_.lookupTransform(baseFrameId_, refCloudFrameId_,
                                                ros::Time(0), transform);
                    pRefLidarDataProcessor_->setDataTransform(
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
                pRefLidarDataProcessor_->setDataTransform(nullptr);
            }
        }
        else
        {
            if (useTfTreeAsInitialGuess_)
                setSensorExtrinsicsFromFrameIds(srcCloudFrameId_, refCloudFrameId_);
        }
    }

    // Level at which to do the processing
    LidarDataProcessor::EProcessingLevel procLevel = (captureCalibrationTarget_)
                                                       ? LidarDataProcessor::TARGET_DETECTION
                                                       : LidarDataProcessor::PREVIEW;

    //--- process data from source lidar asynchronously
    std::future<LidarDataProcessor::EProcessingResult> srcLidarProcFuture =
      std::async(&LidarDataProcessor::processData,
                 pSrcLidarDataProcessor_,
                 srcPointCloud,
                 procLevel);

    //--- process data from reference lidar asynchronously
    std::future<LidarDataProcessor::EProcessingResult> refLidarProcFuture =
      std::async(&LidarDataProcessor::processData,
                 pRefLidarDataProcessor_,
                 refPointCloud,
                 procLevel);

    //--- wait for processing to return
    LidarDataProcessor::EProcessingResult srcLidarProcResult = srcLidarProcFuture.get();
    LidarDataProcessor::EProcessingResult refLidarProcResult = refLidarProcFuture.get();

    //--- if processing level is set to preview, publish preview from each sensor if successful
    if (procLevel == LidarDataProcessor::PREVIEW)
    {
        if (srcLidarProcResult == LidarDataProcessor::SUCCESS)
            pSrcLidarDataProcessor_->publishPreview(ipSrcCloudMsg->header);
        if (refLidarProcResult == LidarDataProcessor::SUCCESS)
            pRefLidarDataProcessor_->publishPreview(ipRefCloudMsg->header.stamp,
                                                    (baseFrameId_.empty())
                                                      ? refFrameId_
                                                      : baseFrameId_);
    }
    //--- else if, processing level is target_detection,
    //--- calibrate only if processing for both sensors is successful
    else if (procLevel == LidarDataProcessor::TARGET_DETECTION)
    {
        if (srcLidarProcResult == LidarDataProcessor::SUCCESS &&
            refLidarProcResult == LidarDataProcessor::SUCCESS)
        {
            //--- publish detections
            pSrcLidarDataProcessor_->publishLastTargetDetection(ipSrcCloudMsg->header);
            pRefLidarDataProcessor_->publishLastTargetDetection(ipRefCloudMsg->header.stamp,
                                                                (baseFrameId_.empty())
                                                                  ? refFrameId_
                                                                  : baseFrameId_);

            //--- perform single calibration iteration
            //--- To this end, this only removes observations without correspondence.
            calibrateLastObservation();
        }
        else
        {
            if (srcLidarProcResult != LidarDataProcessor::SUCCESS &&
                refLidarProcResult == LidarDataProcessor::SUCCESS)
                pRefLidarDataProcessor_->removeCalibIteration(calibrationItrCnt_);

            if (srcLidarProcResult == LidarDataProcessor::SUCCESS &&
                refLidarProcResult != LidarDataProcessor::SUCCESS)
                pSrcLidarDataProcessor_->removeCalibIteration(calibrationItrCnt_);

            CalibrationResultMsg calibResultMsg;
            calibResultMsg.isSuccessful = false;
            calibResultPub_.publish(calibResultMsg);
        }
    }

    //--- if data processor is not pending for more data, set capturing flag to false
    if (srcLidarProcResult != LidarDataProcessor::PENDING &&
        refLidarProcResult != LidarDataProcessor::PENDING)
    {
        captureCalibrationTarget_ = false;
    }
}

//==================================================================================================
bool ExtrinsicLidarLidarCalibrationNodelet::saveCalibrationSettingsToWorkspace()
{
    if (!ExtrinsicCalibrationBase::saveCalibrationSettingsToWorkspace())
        return false;

    QSettings* pCalibSettings = pCalibrationWs_->settingsPtr();
    if (!pCalibSettings)
        return false;

    //--- source lidar sensor name
    pCalibSettings->setValue("source_lidar/sensor_name",
                             QString::fromStdString(srcLidarSensorName_));

    //--- source lidar image topic
    pCalibSettings->setValue("source_lidar/cloud_topic",
                             QString::fromStdString(srcLidarCloudTopic_));

    //--- reference lidar sensor name
    pCalibSettings->setValue("reference_lidar/sensor_name",
                             QString::fromStdString(refLidarSensorName_));

    //--- reference lidar image topic
    pCalibSettings->setValue("reference_lidar/cloud_topic",
                             QString::fromStdString(refLidarCloudTopic_));

    //--- align_ground_planes
    pCalibSettings->setValue("calibration/align_ground_planes",
                             QVariant::fromValue(alignGroundPlanes_));

    //--- upright frame id
    pCalibSettings->setValue("calibration/upright_frame_id",
                             QString::fromStdString(uprightFrameId_));

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
bool ExtrinsicLidarLidarCalibrationNodelet::readLaunchParameters(const ros::NodeHandle& iNh)
{
    if (!ExtrinsicCalibrationBase::readLaunchParameters(iNh))
        return false;

    //--- source_lidar_sensor_name
    srcLidarSensorName_ =
      readStringLaunchParameter(iNh, "src_lidar_sensor_name", DEFAULT_LIDAR_SENSOR_NAME);

    //--- source_lidar_cloud_topic
    srcLidarCloudTopic_ =
      readStringLaunchParameter(iNh, "src_lidar_cloud_topic", DEFAULT_LIDAR_CLOUD_TOPIC);

    //--- ref_lidar_sensor_name
    refLidarSensorName_ =
      readStringLaunchParameter(iNh, "ref_lidar_sensor_name", DEFAULT_LIDAR_SENSOR_NAME);

    //--- reference_lidar_cloud_topic
    refLidarCloudTopic_ =
      readStringLaunchParameter(iNh, "ref_lidar_cloud_topic", DEFAULT_LIDAR_CLOUD_TOPIC);

    //--- align_ground_planes
    iNh.param<bool>("align_ground_planes", alignGroundPlanes_, false);

    //--- upright frame id
    uprightFrameId_ = readStringLaunchParameter(iNh, "upright_frame_id", "");
    if (alignGroundPlanes_ && uprightFrameId_.empty())
    {
        ROS_WARN("[%s] 'align_ground_planes' is activated but 'upright_frame_id' is empty.'"
                 "\nThe alignment of the ground planes will be deactivated."
                 "\nPlease specify a ID of a frame that has an upright z-axes.",
                 nodeletName_.c_str());
    }

    //--- sync queue
    syncQueueSize_ = readNumericLaunchParameter<int>(iNh, "sync_queue_size",
                                                     DEFAULT_SYNC_QUEUE_SIZE, 1, INT_MAX);

    //--- exact sync
    iNh.param<bool>("use_exact_sync", useExactSync_, false);

    return true;
}

//==================================================================================================
void ExtrinsicLidarLidarCalibrationNodelet::reset()
{
    ExtrinsicCalibrationBase::reset();

    pSrcLidarDataProcessor_->reset();
    pSrcLidarDataProcessor_->setPreprocFilter(nullptr);
    pRefLidarDataProcessor_->reset();
    pRefLidarDataProcessor_->setPreprocFilter(nullptr);
}

//==================================================================================================
bool ExtrinsicLidarLidarCalibrationNodelet::shutdownSubscribers()
{
    //--- check if node is initialized
    if (!isInitialized_)
        return false;

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(dataProcessingMutex_);

    //--- unsubscribe subscribers
    srcCloudSubsc_.unsubscribe();
    refCloudSubsc_.unsubscribe();

    return true;
}

} // namespace multisensor_calibration

// Export the class as a plugin using the PLUGINLIB_EXPORT_CLASS macro.
PLUGINLIB_EXPORT_CLASS(multisensor_calibration::ExtrinsicLidarLidarCalibrationNodelet, nodelet::Nodelet)
