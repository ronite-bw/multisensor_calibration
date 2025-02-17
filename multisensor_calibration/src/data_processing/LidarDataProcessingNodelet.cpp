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

#include "../include/multisensor_calibration/data_processing/LidarDataProcessingNodelet.h"

// Std
#include <algorithm>
#include <cmath>
#include <future>
#include <iostream>
#include <numeric>
#include <thread>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>

namespace multisensor_calibration
{

//==================================================================================================
LidarDataProcessingNodelet::LidarDataProcessingNodelet() :
  isInitialized_(false),
  cloudFrameId_(""),
  pLidarDataProcessor_(nullptr),
  captureCalibrationTarget_(false)
{
    //--- connect dyn reconfigure callback
    dynConfigServer_.setCallback(
      boost::bind(&LidarDataProcessingNodelet::dynReconfigureCallback, this, _1, _2));
}

//==================================================================================================
LidarDataProcessingNodelet::~LidarDataProcessingNodelet()
{
}

//==================================================================================================
void LidarDataProcessingNodelet::dynReconfigureCallback(
  LidarDataProcessingNodeletConfig& iConfig, uint32_t level)
{
    UNUSED_VAR(level);

    //--- store object into member
    dynConfig_ = iConfig;

    //--- set to lidar processor
    LidarDataProcessor::setDynConfigParamsToProcessor(dynConfig_, pLidarDataProcessor_.get());
}

//==================================================================================================
bool LidarDataProcessingNodelet::initializeServices(ros::NodeHandle& ioNh)
{
    //--- advertise services

    observSrv_ = ioNh.advertiseService(
      CAPTURE_TARGET_SRV_NAME,
      &LidarDataProcessingNodelet::onRequestTargetCapture, this);

    stateSrv_ = ioNh.advertiseService(
      REQUEST_STATE_SRV_NAME,
      &LidarDataProcessingNodelet::onRequestState, this);

    return true;
}

//==================================================================================================
bool LidarDataProcessingNodelet::initializeSubscribers(ros::NodeHandle& ioNh)
{
    //--- subscribe to topics with name cloudTopic

    cloudSubsc_ = ioNh.subscribe<InputCloud_Message_T>(
      cloudTopicName_, 1, boost::bind(&LidarDataProcessingNodelet::onCloudReceived, this, _1));

    return true;
}

//==================================================================================================
void LidarDataProcessingNodelet::onCloudReceived(
  const InputCloud_Message_T::ConstPtr& ipCloudMsg)
{

#ifdef DEBUG_BUILD
    ROS_INFO("[%s] Message timestamp: %i-%i", __PRETTY_FUNCTION__,
             ipCloudMsg->header.stamp.sec, ipCloudMsg->header.stamp.nsec);
#endif

    //--- check if nodelet is initialized
    if (!isInitialized_ || pLidarDataProcessor_ == nullptr)
    {
        ROS_FATAL("[%s]"
                  "\n\t> Nodelet is not initialized.",
                  Nodelet::getName().c_str());
        return;
    }

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(cloudCallbackMutex_);

    //--- get frame id from header
    cloudFrameId_ = ipCloudMsg->header.frame_id;

    //--- convert input messages to data
    bool isConversionSuccessful = true;

    // point cloud
    pcl::PointCloud<InputPointType> pointCloud;
    isConversionSuccessful &=
      pLidarDataProcessor_->getSensorDataFromMsg(ipCloudMsg, pointCloud);

    if (!isConversionSuccessful)
    {
        ROS_ERROR("[%s] Something went wrong in getting the sensor data from the input messages.",
                  Nodelet::getName().c_str());
        return;
    }

    //--- process data
    LidarDataProcessor::EProcessingLevel procLevel =
      (captureCalibrationTarget_)
        ? LidarDataProcessor::TARGET_DETECTION
        : LidarDataProcessor::PREVIEW;

    //--- process data asynchronously
    std::future<LidarDataProcessor::EProcessingResult> procFuture =
      std::async(&LidarDataProcessor::processData,
                 pLidarDataProcessor_,
                 pointCloud,
                 procLevel);
    LidarDataProcessor::EProcessingResult procResult = procFuture.get();

    if (procResult == LidarDataProcessor::SUCCESS)
    {
        //--- publish regions of interest
        pLidarDataProcessor_->publishPreview(ipCloudMsg->header);

        if (procLevel == LidarDataProcessor::TARGET_DETECTION)
        {
            //--- if calibration target cloud is empty, return and try again
            if (pLidarDataProcessor_->getLastCalibrationTargetCloud()->empty())
            {
                ROS_ERROR("[%s]"
                          "\n\t> Calibration target cloud is empty.",
                          Nodelet::getName()
                            .c_str());
                return;
            }

            //--- if marker corners cloud is empty, return
            if (pLidarDataProcessor_->getLastMarkerCornersCloud()->empty())
            {
                ROS_ERROR("[%s]"
                          "\n\t> Cloud holding 3D points of marker corners is empty.",
                          Nodelet::getName()
                            .c_str());
                return;
            }

            //--- publish result data of target detection
            pLidarDataProcessor_->publishLastTargetDetection(ipCloudMsg->header);
        }
    }

    //--- if data processor is not pending for more data, set capturing flag to false
    if (procResult != LidarDataProcessor::PENDING)
        captureCalibrationTarget_ = false;
}

//==================================================================================================
void LidarDataProcessingNodelet::onInit()
{
#ifdef DEBUG_BUILD
    ROS_INFO("[%s]", __PRETTY_FUNCTION__);
#endif

    //--- get global and private node handle

    // global node handle stored as member variable
    nh_ = getNodeHandle();

    // private node handle used for publishing topics
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    //--- read launch parameters
    isInitialized_ = readLaunchParameters(pnh);

    //--- initialize lidar data processor
    pLidarDataProcessor_.reset(
      new LidarDataProcessor(Nodelet::getName(), "", calibTargetFilePath_));
    if (pLidarDataProcessor_ != nullptr)
    {
        pLidarDataProcessor_->initializePublishers(pnh);
        LidarDataProcessor::setDynConfigParamsToProcessor(dynConfig_, pLidarDataProcessor_.get());
    }
    else
    {
        isInitialized_ = false;
    }

    //--- initialize subscribers
    isInitialized_ &= initializeSubscribers(nh_);

    //--- initialize services
    isInitialized_ &= initializeServices(pnh);
}

//==================================================================================================
bool LidarDataProcessingNodelet::onRequestTargetCapture(
  multisensor_calibration::CaptureCalibTarget::Request& iReq,
  multisensor_calibration::CaptureCalibTarget::Response& oRes)
{
    UNUSED_VAR(iReq);

#ifdef DEBUG_BUILD
    ROS_INFO("[%s]", __PRETTY_FUNCTION__);
#endif

    if (!isInitialized_)
        return false;

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(cloudCallbackMutex_);

    //--- store request
    captureCalibrationTarget_ = true;

    //--- write response
    ROS_INFO("[%s] Start looking for calibration target!",
             Nodelet::getName().c_str());
    oRes.isAccepted = true;
    oRes.msg        = "Start looking for calibration target!";

    return true;
}

//==================================================================================================
bool LidarDataProcessingNodelet::onRequestState(multisensor_calibration::DataProcessorState::Request& iReq,
                                                multisensor_calibration::DataProcessorState::Response& oRes)
{
#ifdef DEBUG_BUILD
    ROS_INFO("[%s]", __PRETTY_FUNCTION__);
#endif

    UNUSED_VAR(iReq);

    //--- store response
    //--- is initialized if internal flag is true and imageFrameId is not empty
    oRes.isInitialized = (isInitialized_ && !cloudFrameId_.empty());
    oRes.frameId       = cloudFrameId_;

    return true;
}

//==================================================================================================
bool LidarDataProcessingNodelet::readLaunchParameters(const ros::NodeHandle& iNh)
{
    // topic name of input cloud
    iNh.param<std::string>("cloud", cloudTopicName_, std::string("/cloud"));

    // path to target configuration file
    std::string targetFileStr;
    iNh.param<std::string>("target_config_file", targetFileStr, std::string(""));
    if (targetFileStr.empty() || !fs::exists(targetFileStr))
    {
        ROS_FATAL("[%s]"
                  "\n\t> Target configuration file path is empty or does not consist: %s",
                  Nodelet::getName().c_str(), targetFileStr.c_str());
        return false;
    }
    calibTargetFilePath_ = fs::absolute(targetFileStr);

    return true;
}

} // namespace multisensor_calibration

// Export the class as a plugin using the PLUGINLIB_EXPORT_CLASS macro.
PLUGINLIB_EXPORT_CLASS(multisensor_calibration::LidarDataProcessingNodelet, nodelet::Nodelet)