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

#ifndef MULTISENSORCALIBRATION_LIDARDATAPROCESSINGNODELET_H
#define MULTISENSORCALIBRATION_LIDARDATAPROCESSINGNODELET_H

// Std
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

// Eigen
#include <Eigen/Geometry>

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OpenCV
#include <opencv2/core.hpp>

// multisensor_calibration
#include "../common/common.h"
#include "../common/lib3D/core/camera.hpp"
#include "LidarDataProcessor.h"
#include <multisensor_calibration/CaptureCalibTarget.h>
#include <multisensor_calibration/DataProcessorState.h>
#include <multisensor_calibration/LidarDataProcessingNodeletConfig.h>

namespace fs = std::filesystem;

namespace multisensor_calibration
{

/**
 * @brief Nodelet to run the processing of the LiDAR data and, in turn, the detection of the
 * calibration target within the LiDAR point clouds isolated from the rest.
 *
 * <b>Launch-Parameters:</b>
 * - ```cloud```: Topic name of the LiDAR cloud messages in which the target is to be detected.
 *    - Type: String
 *    - Default: "/cloud"
 * - ```target_config_file```: Path to the file holding the configuration of the calibration target.
 *    E.g. \"$(find multisensor_calibration)/config/TargetWithCirclesAndAruco.yaml\"
 *
 * <b>Topics Published:</b>
 * - ```/<nodelet_name>/regions_of_interest```: Cloud holding separate regions of
 *    the input sensor cloud in which the calibration target is searched for. These are the product
 *    of the first preprocessing of the sensor cloud to reduce the amount of data to be processed.
 * - ```/<nodelet_name>/target_pattern```: Cloud holding points of the calibration
 *    target detected in the LiDAR cloud. This is only available after the calibration target has
 *    been detected in the LiDAR cloud.
 * - ```/<nodelet_name>/marker_corners```: Corners of the ArUco markers on the
 *    calibration target deduced from the detected pose of the target. Each point of he marker
 *    corner is enhanced with the ID of the ArUco marker inside the ```intensity``` field. This is
 *    only available after the calibration target has been detected in the LiDAR cloud.
 * - ```/<nodelet_name>/board_pose```: 6-DOF pose of the detected calibration
 *    target. This is only available after the calibration target has been detected in the LiDAR
 *    cloud.
 *
 * <b>Services Advertised:</b>
 *  - ```/<nodelet_name>/capture_target```: Service to trigger the capturing of the target in both
 *    sensors. This will add an observation to the list, if the target detection was successful.
 *  - ```/<nodelet_name>/request_processor_state```: Service to get initialization state of the
 *    processor.
 *
 */
class LidarDataProcessingNodelet : public nodelet::Nodelet
{

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Default Constructor
     */
    LidarDataProcessingNodelet();

    /**
     * @brief Default Destructor
     */
    virtual ~LidarDataProcessingNodelet();

  private:
    /**
     * @brief Callback function handling parameter changes by dynamic reconfigure.
     *
     * @param[in] iConfig Config object
     * @param[in] level
     */
    void dynReconfigureCallback(LidarDataProcessingNodeletConfig& iConfig, uint32_t level);

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
     * @brief Callback function handling point cloud messages.
     *
     * @param[in] ipCloudMsg cloud message.
     */
    void onCloudReceived(const InputCloud_Message_T::ConstPtr& ipCloudMsg);

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
     * @brief Service call to request capturing of calibration target
     *
     * @param[in] iReq Request.
     * @param[out] oRes Response.
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

    /// Mutex guarding the cloud callback
    std::mutex cloudCallbackMutex_;

    /// Global node handler
    ros::NodeHandle nh_;

    /// Dynamic reconfigure server
    dynamic_reconfigure::Server<LidarDataProcessingNodeletConfig> dynConfigServer_;

    /// Dynamic reconfigure object
    LidarDataProcessingNodeletConfig dynConfig_;

    /// Server to provide service to request observation of marker corners
    ros::ServiceServer observSrv_;

    /// Server to provide service to request the processor state
    ros::ServiceServer stateSrv_;

    /// Subscriber to point cloud topic
    ros::Subscriber cloudSubsc_;

    /// Path to target configuration file
    fs::path calibTargetFilePath_;

    /// Name of clout to which to subscribe
    std::string cloudTopicName_;

    /// Frame id of cloud received by #cloudSubsc_
    std::string cloudFrameId_;

    /// Pointer to object of lidar data processor, responsible to detect calibration target
    /// in lidar cloud data.
    std::shared_ptr<LidarDataProcessor> pLidarDataProcessor_;

    /// Flag to capture calibration target in next sensor data package.
    bool captureCalibrationTarget_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_LIDARDATAPROCESSINGNODELET_H
