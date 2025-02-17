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

#include "multisensor_calibration/calibration/ExtrinsicLidarReferenceCalibrationNodelet.h"

// Std
#include <fstream>
#include <functional>
#include <future>
#include <thread>

// PCL
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
#include "multisensor_calibration/common/common.h"
#include "multisensor_calibration/common/utils.hpp"

namespace multisensor_calibration
{

//==================================================================================================
ExtrinsicLidarReferenceCalibrationNodelet::ExtrinsicLidarReferenceCalibrationNodelet() :
  Extrinsic3d3dCalibrationBase<LidarDataProcessor, ReferenceDataProcessor3d>(EXTRINSIC_LIDAR_REFERENCE_CALIBRATION)
{
    //--- connect dyn reconfigure callback
    dynConfigServer_.setCallback(
      boost::bind(&ExtrinsicLidarReferenceCalibrationNodelet::dynReconfigureCallback,
                  this, _1, _2));
}

//==================================================================================================
ExtrinsicLidarReferenceCalibrationNodelet::~ExtrinsicLidarReferenceCalibrationNodelet()
{

    pSrcLidarDataProcessor_.reset();
    pRefDataProcessor_.reset();
}

//==================================================================================================
void ExtrinsicLidarReferenceCalibrationNodelet::dynReconfigureCallback(
  ExtrinsicLidarReferenceCalibrationConfig& iConfig, uint32_t level)
{
    UNUSED_VAR(level);

    //--- store object into member
    dynConfig_ = iConfig;

    //--- set to lidar processors
    LidarDataProcessor::setDynConfigParamsToProcessor(dynConfig_, pSrcLidarDataProcessor_.get());
}

//==================================================================================================
void ExtrinsicLidarReferenceCalibrationNodelet::finalizeCalibration()
{

    //--- calculate coarse extrinsic pose based on the marker observations
    //--- in case of LiDAR-LiDAR calibration, this is done after the detection of the target
    //--- when a single iteration is calibrated
    //--- but since, this is not done in case of the LiDAR-Reference calibration, it needs to
    //--- be done prior to the final calibration

    //--- copy marker corner observations to point cloud
    //--- make uneven in order to remove ambiguities
    pcl::PointCloud<pcl::PointXYZ>::Ptr pSrcMarkerCornerCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pRefMarkerCornerCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (uint i = 1; i < calibrationItrCnt_; ++i)
    {
        //--- get observations from source LiDAR
        std::set<uint> srcObservationIds;
        std::vector<cv::Point3f> srcCornerObservations;
        pSrcLidarDataProcessor_->getOrderedObservations(srcObservationIds, srcCornerObservations,
                                                        i, 1);

        //--- get observations from reference LiDAR
        std::set<uint> refObservationIds;
        std::vector<cv::Point3f> refCornerObservations;
        pRefDataProcessor_->getOrderedObservations(refObservationIds, refCornerObservations,
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

    if (pSrcMarkerCornerCloud->empty() || pRefMarkerCornerCloud->empty())
    {
        ROS_ERROR("[%s] Could not finalize calibration. No common observations available.",
                  nodeletName_.c_str());
        return;
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

    //--- do fine calibration with icp
    //--- in this, first merge target clouds

    // pointer to cloud holding calibration target clouds of all observations from the src lidar
    pcl::PointCloud<InputPointType>::Ptr pSrcLidarTargetClouds(
      new pcl::PointCloud<InputPointType>());

    // pointer to cloud holding calibration target clouds of all observations from the ref lidar
    pcl::PointCloud<InputPointType>::Ptr pRefTargetClouds(
      new pcl::PointCloud<InputPointType>());

    auto srcLidarTargetCloudPtrs = pSrcLidarDataProcessor_->getCalibrationTargetCloudPtrs();
    for (pcl::PointCloud<InputPointType>::Ptr pSubCloud : srcLidarTargetCloudPtrs)
        pSrcLidarTargetClouds->insert(pSrcLidarTargetClouds->end(),
                                      pSubCloud->begin(),
                                      pSubCloud->end());
    auto refLidarTargetCloudPtrs = pRefDataProcessor_->getCalibrationTargetCloudPtrs();
    for (pcl::PointCloud<InputPointType>::Ptr pSubCloud : refLidarTargetCloudPtrs)
        pRefTargetClouds->insert(pRefTargetClouds->end(),
                                 pSubCloud->begin(),
                                 pSubCloud->end());

    //--- run ICP
    double icpRmse = runIcp<InputPointType>(
      pSrcLidarTargetClouds, pRefTargetClouds,
      static_cast<small_gicp::RegistrationSetting::RegistrationType>(dynConfig_.registration_icp_variant),
      dynConfig_.registration_icp_max_correspondence_distance,
      dynConfig_.registration_icp_rotation_tolerance,
      dynConfig_.registration_icp_translation_tolerance);
    calibResult_.error = std::make_pair("Root Mean Squared Error (in m)", icpRmse);

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
bool ExtrinsicLidarReferenceCalibrationNodelet::initializeDataProcessors()
{
    bool isSuccessful = true;

    //--- initialize source lidar data processor
    pSrcLidarDataProcessor_.reset(
      new LidarDataProcessor(nodeletName_, srcSensorName_, calibTargetFilePath_));
    if (pSrcLidarDataProcessor_)
    {
        pSrcLidarDataProcessor_->initializeServices(pnh_);
        pSrcLidarDataProcessor_->initializePublishers(pnh_);
        LidarDataProcessor::setDynConfigParamsToProcessor(dynConfig_,
                                                          pSrcLidarDataProcessor_.get());
    }
    else
    {
        isSuccessful = false;
    }

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
bool ExtrinsicLidarReferenceCalibrationNodelet::initializeSubscribers(ros::NodeHandle& ioNh)
{
    //--- subscribe to topic from source sensor
    srcCloudSubsc_ = ioNh.subscribe(
      srcLidarCloudTopic_, 1,
      &ExtrinsicLidarReferenceCalibrationNodelet::onSensorDataReceived, this);

    return true;
}

//==================================================================================================
bool ExtrinsicLidarReferenceCalibrationNodelet::initializeWorkspaceObjects()
{
    bool retVal = true;

    retVal &= CalibrationBase::initializeWorkspaceObjects();

    //--- initialize calibration workspace
    fs::path calibWsPath = robotWsPath_;
    calibWsPath /=
      std::string(srcLidarSensorName_ + "_" + refSensorName_ + "_extrinsic_calibration");
    pCalibrationWs_ =
      std::make_shared<ExtrinsicLidarReferenceCalibWorkspace>(calibWsPath, nodeletName_);
    retVal &= (pCalibrationWs_ != nullptr);

    //--- copy calibration target config to workspace
    if (pCalibrationWs_)
    {
        //--- create backup
        retVal &= pCalibrationWs_->createBackup();
    }

    return retVal;
}

//==================================================================================================
void ExtrinsicLidarReferenceCalibrationNodelet::onInit()
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
bool ExtrinsicLidarReferenceCalibrationNodelet::onRequestRemoveObservation(
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

        //--- since no per iteration calibration is performed no sensorExtrinsic needs
        //-- to be removed
        // sensorExtrinsics_.pop_back();

        oRes.isAccepted = true;
        oRes.msg        = "Last observation successfully removed!"
                          "\n\t> Remaining number of observations: " +
                   std::to_string(pSrcDataProcessor_->getNumCalibIterations()) + " (src), " +
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
void ExtrinsicLidarReferenceCalibrationNodelet::onSensorDataReceived(
  const InputCloud_Message_T::ConstPtr& ipSrcCloudMsg)
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

    // source point cloud
    pcl::PointCloud<InputPointType> srcPointCloud;
    isConversionSuccessful &=
      pSrcLidarDataProcessor_->getSensorDataFromMsg(ipSrcCloudMsg, srcPointCloud);

    if (!isConversionSuccessful)
    {
        ROS_ERROR("[%s] Something went wrong in getting the sensor data from the input messages.",
                  nodeletName_.c_str());
        return;
    }

    //--- set frame ids and initialize sensor extrinsics if applicable
    if (srcCloudFrameId_ != ipSrcCloudMsg->header.frame_id)
    {
        srcCloudFrameId_ = ipSrcCloudMsg->header.frame_id;

        //--- compute sensor extrinsics between source frame id and ref or base frame id
        //--- if base frame id is not empty and unequal to refCloudFrameId also get transform
        //--- between refFrameID and baseFrameID and pass to refLidarProcessor.
        if (!baseFrameId_.empty() && baseFrameId_ != refFrameId_)
        {
            if (useTfTreeAsInitialGuess_)
                setSensorExtrinsicsFromFrameIds(srcCloudFrameId_, baseFrameId_);

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
                setSensorExtrinsicsFromFrameIds(srcCloudFrameId_, refFrameId_);
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

    //--- wait for processing to return
    LidarDataProcessor::EProcessingResult srcLidarProcResult = srcLidarProcFuture.get();

    //--- if processing level is set to preview, publish preview from each sensor if successful
    if (procLevel == LidarDataProcessor::PREVIEW)
    {
        if (srcLidarProcResult == LidarDataProcessor::SUCCESS)
            pSrcLidarDataProcessor_->publishPreview(ipSrcCloudMsg->header);
    }
    //--- else if, processing level is target_detection,
    //--- calibrate only if processing for both sensors is successful
    else if (procLevel == LidarDataProcessor::TARGET_DETECTION)
    {
        CalibrationResultMsg calibResultMsg;

        if (srcLidarProcResult == LidarDataProcessor::SUCCESS)
        {
            //--- publish detections
            pSrcLidarDataProcessor_->publishLastTargetDetection(ipSrcCloudMsg->header);

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
    if (srcLidarProcResult != LidarDataProcessor::PENDING)
    {
        captureCalibrationTarget_ = false;
    }
}

//==================================================================================================
bool ExtrinsicLidarReferenceCalibrationNodelet::saveCalibrationSettingsToWorkspace()
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
bool ExtrinsicLidarReferenceCalibrationNodelet::readLaunchParameters(const ros::NodeHandle& iNh)
{
    if (!ExtrinsicCalibrationBase::readLaunchParameters(iNh))
        return false;

    //--- source_lidar_sensor_name
    srcLidarSensorName_ =
      readStringLaunchParameter(iNh, "src_lidar_sensor_name", DEFAULT_LIDAR_SENSOR_NAME);

    //--- source_lidar_cloud_topic
    srcLidarCloudTopic_ =
      readStringLaunchParameter(iNh, "src_lidar_cloud_topic", DEFAULT_LIDAR_CLOUD_TOPIC);

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
void ExtrinsicLidarReferenceCalibrationNodelet::reset()
{
    ExtrinsicCalibrationBase::reset();

    pSrcLidarDataProcessor_->reset();
    pSrcLidarDataProcessor_->setPreprocFilter(nullptr);
    pRefDataProcessor_->reset();
}

//==================================================================================================
bool ExtrinsicLidarReferenceCalibrationNodelet::shutdownSubscribers()
{
    //--- check if node is initialized
    if (!isInitialized_)
        return false;

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(dataProcessingMutex_);

    //--- unsubscribe subscribers
    srcCloudSubsc_.shutdown();

    return true;
}

} // namespace multisensor_calibration

// Export the class as a plugin using the PLUGINLIB_EXPORT_CLASS macro.
PLUGINLIB_EXPORT_CLASS(multisensor_calibration::ExtrinsicLidarReferenceCalibrationNodelet,
                       nodelet::Nodelet)
