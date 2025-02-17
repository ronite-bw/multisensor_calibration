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

#include "../include/multisensor_calibration/data_processing/CameraDataProcessingNodelet.h"

// Std
#include <cmath>
#include <future>
#include <iostream>
#include <thread>
#include <type_traits>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>

// OpenCV
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// multisensor_calibration
#include "../include/multisensor_calibration/common/utils.hpp"
#include "../include/multisensor_calibration/config/CalibrationTarget.hpp"
#define NO_QT
#include "../include/multisensor_calibration/common/lib3D/core/camera.hpp"
#include "../include/multisensor_calibration/common/lib3D/core/geometry.hpp"
#include "../include/multisensor_calibration/common/lib3D/core/image.hpp"
#include "../include/multisensor_calibration/common/lib3D/core/visualization.hpp"

namespace multisensor_calibration
{

//==================================================================================================
CameraDataProcessingNodelet::CameraDataProcessingNodelet() :
  isInitialized_(false),
  cameraNamespace_(""),
  imageName_(""),
  imageFrameId_(""),
  pCamDataProcessor_(nullptr),
  imageState_(STR_2_IMG_STATE.at(DEFAULT_IMG_STATE_STR)),
  captureCalibrationTarget_(false)
{
}

//==================================================================================================
CameraDataProcessingNodelet::~CameraDataProcessingNodelet()
{
}

//==================================================================================================
bool CameraDataProcessingNodelet::initializeServices(ros::NodeHandle& ioNh)
{
    //--- advertise services

    cameraIntrSrv_ = ioNh.advertiseService(
      REQUEST_CAM_INTRINSICS_SRV_NAME,
      &CameraDataProcessingNodelet::onRequestCameraIntrinsics, this);

    captureSrv_ = ioNh.advertiseService(
      CAPTURE_TARGET_SRV_NAME,
      &CameraDataProcessingNodelet::onRequestTargetCapture, this);

    stateSrv_ = ioNh.advertiseService(
      REQUEST_STATE_SRV_NAME,
      &CameraDataProcessingNodelet::onRequestState, this);

    return true;
}

//==================================================================================================
bool CameraDataProcessingNodelet::initializeSubscribers(ros::NodeHandle& ioNh)
{
    //--- subscribe to topics with name constructed from camera Namespace and imageName

    image_transport::ImageTransport imgTransp(ioNh);
    imageSubsc_ = imgTransp.subscribe(
      cameraNamespace_ + "/" + imageName_, 10,
      boost::bind(&CameraDataProcessingNodelet::onImageReceived,
                  this, _1));

    //--- get camera intrinsics from camera_info

    // pointer to camera info message
    sensor_msgs::CameraInfoConstPtr camInfo =
      ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cameraNamespace_ + "/camera_info",
                                                          ros::Duration(20, 0));
    if (camInfo != nullptr)
    {
        lib3d::Intrinsics cameraIntr;
        utils::setCameraIntrinsicsFromCameraInfo(*camInfo.get(),
                                                 cameraIntr,
                                                 imageState_);
        if (pCamDataProcessor_)
            pCamDataProcessor_->setCameraIntrinsics(cameraIntr);
    }
    else
    {
        ROS_FATAL("[%s]"
                  "\n\t> Wait for message of camera_info topic has timed out. "
                  "\n\t> Camera intrinsics will not be set!",
                  Nodelet::getName().c_str());
        return false;
    }

    return true;
}

//==================================================================================================
void CameraDataProcessingNodelet::onImageReceived(
  const InputImage_Message_T::ConstPtr& ipImgMsg)
{

#ifdef DEBUG_BUILD
    ROS_INFO("[%s] | %i-%i", __PRETTY_FUNCTION__,
             ipImgMsg->header.stamp.sec, ipImgMsg->header.stamp.nsec);
#endif

    //--- check if nodelet is initialized
    if (!isInitialized_ || pCamDataProcessor_ == nullptr)
    {
        ROS_FATAL("[%s] Nodelet is not initialized!",
                  Nodelet::getName().c_str());
        return;
    }

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(imageCallbackMutex_);

    //--- get frame id from header
    imageFrameId_ = ipImgMsg->header.frame_id;

    //--- convert input messages to data
    bool isConversionSuccessful = true;

    // camera image
    cv::Mat cameraImage;
    isConversionSuccessful &= pCamDataProcessor_->getSensorDataFromMsg(ipImgMsg, cameraImage);

    if (!isConversionSuccessful)
    {
        ROS_ERROR("[%s] Something went wrong in getting the sensor data from the input messages.",
                  Nodelet::getName().c_str());
        return;
    }

    //--- process data
    CameraDataProcessor::EProcessingLevel procLevel =
      (captureCalibrationTarget_)
        ? CameraDataProcessor::TARGET_DETECTION
        : CameraDataProcessor::PREVIEW;

    //--- process data asynchronously
    std::future<CameraDataProcessor::EProcessingResult> procFuture =
      std::async(&CameraDataProcessor::processData,
                 pCamDataProcessor_,
                 cameraImage,
                 procLevel);
    CameraDataProcessor::EProcessingResult procResult = procFuture.get();

    if (procResult == CameraDataProcessor::SUCCESS)
    {
        //--- publish preview data, i.e. annotated camera image
        pCamDataProcessor_->publishPreview(ipImgMsg->header);

        if (procLevel == CameraDataProcessor::TARGET_DETECTION)
        {

            //--- if calibration target cloud is empty, return and try again
            if (pCamDataProcessor_->getLastCalibrationTargetCloudPtr()->empty())
            {
                ROS_ERROR("[%s]"
                          "\n\t> Calibration target cloud is empty.",
                          Nodelet::getName()
                            .c_str());
                return;
            }

            //--- publish result data of target detection
            pCamDataProcessor_->publishLastTargetDetection(ipImgMsg->header);
        }
    }

    //--- if data processor is not pending for more data, set capturing flag to false
    if (procResult != CameraDataProcessor::PENDING)
        captureCalibrationTarget_ = false;
}

//==================================================================================================
void CameraDataProcessingNodelet::onInit()
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

    //--- initialize camera data processor
    pCamDataProcessor_.reset(new CameraDataProcessor(Nodelet::getName(), "", calibTargetFilePath_));
    if (pCamDataProcessor_)
    {
        pCamDataProcessor_->setImageState(imageState_);
        pCamDataProcessor_->initializePublishers(pnh);
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
bool CameraDataProcessingNodelet::onRequestCameraIntrinsics(
  CameraIntrinsics::Request& iReq,
  CameraIntrinsics::Response& oRes)
{
#ifdef DEBUG_BUILD
    ROS_INFO("[%s]", __PRETTY_FUNCTION__);
#endif

    UNUSED_VAR(iReq);

    if (!isInitialized_ || pCamDataProcessor_ == nullptr)
        return false;

    lib3d::Intrinsics cameraIntr = pCamDataProcessor_->getCameraIntrinsics();

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

    return true;
}

//==================================================================================================
bool CameraDataProcessingNodelet::onRequestTargetCapture(
  multisensor_calibration::CaptureCalibTarget::Request& iReq,
  multisensor_calibration::CaptureCalibTarget::Response& oRes)
{
    UNUSED_VAR(iReq);

#ifdef DEBUG_BUILD
    ROS_INFO("[%s]", __PRETTY_FUNCTION__);
#endif

    UNUSED_VAR(oRes);

    if (!isInitialized_)
        return false;

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(imageCallbackMutex_);

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
bool CameraDataProcessingNodelet::onRequestState(multisensor_calibration::DataProcessorState::Request& iReq,
                                                 multisensor_calibration::DataProcessorState::Response& oRes)
{
#ifdef DEBUG_BUILD
    ROS_INFO("[%s]", __PRETTY_FUNCTION__);
#endif

    UNUSED_VAR(iReq);

    //--- store response
    //--- is initialized if internal flag is true and imageFrameId is not empty
    oRes.isInitialized = (isInitialized_ && !imageFrameId_.empty());
    oRes.frameId       = imageFrameId_;

    return true;
}

//==================================================================================================
bool CameraDataProcessingNodelet::readLaunchParameters(const ros::NodeHandle& iNh)
{
    // namespace of the camera to which to subscribe
    iNh.param<std::string>("camera", cameraNamespace_, std::string("/camera"));
    if (cameraNamespace_.front() != '/' && !cameraNamespace_.empty())
        cameraNamespace_ = "/" + cameraNamespace_; // prepend slash to namespace, if need be
    while (cameraNamespace_.back() == '/' && !cameraNamespace_.empty())
        cameraNamespace_.pop_back(); // remove slash(s) from back of namespace, if need be

    // name of image (within cameraNamespace) to subscribe to
    iNh.param<std::string>("image", imageName_, std::string("image_color"));
    while (imageName_.front() == '/' && !imageName_.empty())
        imageName_ = imageName_.substr(1); // remove slash(s) from from of imageName, if need be
    while (imageName_.back() == '/' && !imageName_.empty())
        imageName_.pop_back(); // remove slash(s) from from of imageName, if need be

    // image state
    std::string imageStateStr;
    iNh.param<std::string>("image_state", imageStateStr, DEFAULT_IMG_STATE_STR);
    auto findItr = STR_2_IMG_STATE.find(imageStateStr);
    if (findItr != STR_2_IMG_STATE.end())
        imageState_ = findItr->second;

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
PLUGINLIB_EXPORT_CLASS(multisensor_calibration::CameraDataProcessingNodelet, nodelet::Nodelet)