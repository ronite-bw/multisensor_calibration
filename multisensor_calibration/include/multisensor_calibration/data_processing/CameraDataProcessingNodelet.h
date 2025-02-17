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

#ifndef MULTISENSORCALIBRATION_CAMERADATAPROCESSINGNODELET_H
#define MULTISENSORCALIBRATION_CAMERADATAPROCESSINGNODELET_H

// Std
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

// ROS
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// multisensor_calibration
#include "../common/common.h"
#include "CameraDataProcessor.h"
#include <multisensor_calibration/CameraIntrinsics.h>
#include <multisensor_calibration/CaptureCalibTarget.h>
#include <multisensor_calibration/DataProcessorState.h>

namespace fs = std::filesystem;

namespace multisensor_calibration
{

/**
 * @brief Nodelet to run the processing of the camera data and, in turn, the detection of the
 * calibration target within the camera data isolated from the rest.
 */
class CameraDataProcessingNodelet : public nodelet::Nodelet
{
    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Default Constructor
     */
    CameraDataProcessingNodelet();

    /**
     * @brief Default Destructor
     */
    virtual ~CameraDataProcessingNodelet();

  private:
    /**
     * @brief Method to initialize services
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    bool initializeServices(ros::NodeHandle& ioNh);

    /**
     * @brief Method to initialize subscribers
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    bool initializeSubscribers(ros::NodeHandle& ioNh);

    /**
     * @brief Callback function handling image messages.
     *
     * @param[in] ipImgMsg image message.
     */
    void onImageReceived(const InputImage_Message_T::ConstPtr& ipImgMsg);

    /**
     * @overload
     * @brief The onInit method is called by nodelet manager. It is responsible for initializing the
     * nodelet.
     *
     * @note It is important that this method returns, otherwise the nodelet gets stuck during the
     * initialization
     */
    void onInit() override;

    /**
     * @brief Service call to get camera intrinsics.
     *
     * @param[in] iReq Request, UNUSED.
     * @param[out] oRes Response.
     */
    bool onRequestCameraIntrinsics(multisensor_calibration::CameraIntrinsics::Request& iReq,
                                   multisensor_calibration::CameraIntrinsics::Response& oRes);

    /**
     * @brief Service call to request capturing of calibration target
     *
     * @param[in] iReq Request, with flag to capture calibration target
     * @param[out] oRes Response, empty.
     */
    bool onRequestTargetCapture(multisensor_calibration::CaptureCalibTarget::Request& iReq,
                                multisensor_calibration::CaptureCalibTarget::Response& oRes);

    /**
     * @brief Service call to request state of of the data processor
     *
     * @param[in] iReq Request, UNUSED.
     * @param[out] oRes Response, holding the initialization state and the frame id of the data
     * processed.
     */
    bool onRequestState(multisensor_calibration::DataProcessorState::Request& iReq,
                        multisensor_calibration::DataProcessorState::Response& oRes);

    /**
     * @brief Method to read launch parameters.
     *
     * @param[in] iNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    bool readLaunchParameters(const ros::NodeHandle& iNh);

    //--- MEMBER DECLARATION ---//

  private:
    /// Flag indicating if nodelet is initialized
    bool isInitialized_;

    /// Mutex guarding the image callback
    std::mutex imageCallbackMutex_;

    /// Global node handler
    ros::NodeHandle nh_;

    /// Server to provide service to get camera intrinsics
    ros::ServiceServer cameraIntrSrv_;

    /// Server to provide service to request observation of marker corners
    ros::ServiceServer captureSrv_;

    /// Server to provide service to request the processor state
    ros::ServiceServer stateSrv_;

    /// Subscriber to image topic
    image_transport::Subscriber imageSubsc_;

    /// Subscriber to camera info topic
    ros::Subscriber camInfoSubsc_;

    /// Path to target configuration file
    fs::path calibTargetFilePath_;

    /// Namespace of camera to which to subscribe
    std::string cameraNamespace_;

    /// Name of image to which to subscribe within cameraNamespace_
    std::string imageName_;

    /// Frame id of image received by #imageSubsc_
    std::string imageFrameId_;

    /// Pointer to object of camera data processor, responsible to detect calibration target
    /// in camera image data.
    std::shared_ptr<CameraDataProcessor> pCamDataProcessor_;

    /// State of image received on the subscribed topic
    EImageState imageState_;

    /// Flag to capture calibration target in next sensor data package.
    bool captureCalibrationTarget_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_CAMERADATAPROCESSINGNODELET_H
