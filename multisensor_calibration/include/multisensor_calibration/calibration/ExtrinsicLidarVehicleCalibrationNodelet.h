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

#ifndef MULTISENSORCALIBRATION_EXTRINSICLIDARVEHICLECALIBRATIONNODELET_H
#define MULTISENSORCALIBRATION_EXTRINSICLIDARVEHICLECALIBRATIONNODELET_H

// Std
#include <memory>
#include <string>
#include <tuple>

// ROS
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <nodelet/nodelet.h>
#include <ros/service.h>
#include <ros/subscriber.h>

// OpenCV
#include <opencv2/core.hpp>

// multisensor_calibration
#include "../common/common.h"
#include "Extrinsic3d3dCalibrationBase.h"
#include <multisensor_calibration/AddRegionMarker.h>
#include <multisensor_calibration/ExtrinsicLidarVehicleCalibrationConfig.h>

// #define USE_NORMAL_INFO

namespace multisensor_calibration
{

/**
 * @brief Nodelet to perform extrinsic lidar-vehicle calibration.
 *
 * This subclasses multisensor_calibration::Extrinsic3d3dCalibrationBase.
 */
class ExtrinsicLidarVehicleCalibrationNodelet
  : public Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>,
    public nodelet::Nodelet
{

#ifdef USE_NORMAL_INFO
    using RegionPointType = pcl::PointXYZINormal;
#else
    using RegionPointType = InputPointType;
#endif

    //--- METHOD DECLARATION ---/
  public:
    /**
     * @brief Default constructor
     */
    ExtrinsicLidarVehicleCalibrationNodelet();

    /**
     * @brief Destructor
     */
    virtual ~ExtrinsicLidarVehicleCalibrationNodelet();

  private:
    /**
     * @brief Compute planar regions in cloud data based on selected region markers
     *
     * @param[in] ipCloudMsg Pointer to cloud message from which the regions cloud is to be computed.
     * @param[in] iRegionMarkers List of markers for which regions should be computed.
     * @param[out] oRegionSeedPointsPtrs List of point clouds holding the seed points for the
     * selected regions.
     * @param[out] opRegionsCloud Pointer to cloud which is to hold the regions.
     */
    void computeRegionsCloud(const InputCloud_Message_T::ConstPtr& ipCloudMsg,
                             const std::vector<InputPointType>& iRegionMarkers,
                             std::vector<pcl::PointCloud<InputPointType>::Ptr>& oRegionSeedPointsPtrs,
                             pcl::PointCloud<RegionPointType>::Ptr& opRegionsCloud) const;

    /**
     * @brief Do coarse calibration based on the seed points of the regions of interest.
     *
     * This will estimate a rigid transformation between the selected seed points.
     */
    void doCoarseCalibration();

    /**
     * @brief Callback function handling parameter changes by dynamic reconfigure.
     *
     * @param[in] iConfig Config object
     * @param[in] level
     */
    void dynReconfigureCallback(ExtrinsicLidarVehicleCalibrationConfig& iConfig, uint32_t level);

    /**
     * @brief Method to initialize subscribers. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    void finalizeCalibration() override;

    /**
     * @brief Method to initialize publishers. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    bool initializePublishers(ros::NodeHandle& ioNh) override;

    /**
     * @brief Method to initialize services. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    bool initializeServices(ros::NodeHandle& ioNh) override;

    /**
     * @brief Method to initialize subscribers. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    bool initializeSubscribers(ros::NodeHandle& ioNh) override;

    /**
     * @brief Method to initialize workspace objects. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * In this class, the object of the calibration
     * workspace is initialized. The initialization requires the launch parameters, thus it is to
     * be executed after the launch parameters are read.
     *
     * @return True if successful. False, otherwise (e.g. if instantiation has failed)
     */
    bool initializeWorkspaceObjects() override;

    /**
     * @brief The onInit method is called by nodelet manager. It is responsible for initializing the
     * nodelet.
     *
     * @note It is important that this method returns, otherwise the nodelet gets stuck during the
     * initialization
     */
    void onInit() override;

    /**
     * @brief Handle point clicked in RViz
     *
     * @param[in] ipPointMsg Pointer to message holding the clicked point.
     */
    void onPointClicked(const geometry_msgs::PointStampedConstPtr& ipPointMsg);

    /**
     * @brief handle service call to request addition of region marker.
     *
     * @param[in] iReq Request
     * @param[out] oRes Response
     */
    bool onRequestAddRegionMarker(
      multisensor_calibration::AddRegionMarker::Request& iReq,
      multisensor_calibration::AddRegionMarker::Response& oRes);

    /**
     * @brief Handle service call to request removing of last observation.
     *
     * @param[in] iReq Request, with flag to capture calibration target
     * @param[out] oRes Response, empty.
     */
    bool onRequestRemoveObservation(
      multisensor_calibration::RemoveLastObservation::Request& iReq,
      multisensor_calibration::RemoveLastObservation::Response& oRes) override;

    /**
     * @brief Method to receive reference data from vehicle model.
     *
     * @param[in] ipRefCloudMsg Pointer to point cloud message from reference model.
     */
    void onReferenceDataReceived(
      const InputCloud_Message_T::ConstPtr& ipRefCloudMsg);

    /**
     * @brief Method to receive sensor data from source LiDAR which is to be calibrated with respect
     * to the vehicle.
     *
     * @param[in] ipSrcCloudMsg Pointer to point cloud message from source sensor that is to be
     * calibrated.
     */
    void onSensorDataReceived(
      const InputCloud_Message_T::ConstPtr& ipSrcCloudMsg);

    /**
     * @brief Method to save calibration specific settings to the workspace. This overrides the
     * method of the parent class.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    bool saveCalibrationSettingsToWorkspace() override;

    /**
     * @brief Method to read launch parameters. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * @param[in] iNh Object of node handle
     * @return True if successful. False, otherwise (e.g. if sanity check fails)
     */
    bool readLaunchParameters(const ros::NodeHandle& iNh) override;

    /**
     * @brief Method to reset calibration. This overrides the method of the parent class.
     * In this, the parent method is also called.
     */
    void reset() override;

    /**
     * @brief Method to shutdown subscribers and disconnect callbacks. This overrides the method of
     * the parent class. In this, the parent method is also called.
     *
     * @return True, if successful. False, otherwise.
     */
    bool shutdownSubscribers() override;

    //--- MEMBER DECLARATION ---/

  private:
    /// Dynamic reconfigure server
    dynamic_reconfigure::Server<ExtrinsicLidarVehicleCalibrationConfig> dynConfigServer_;

    /// Object of region growing parameters from dynamic reconfigure config
    ExtrinsicLidarVehicleCalibrationConfig dynConfig_;

    /// Node handle
    using CalibrationBase::nh_;

    /// Mutex guarding the processing of sensor data.
    /// This is a reference to CalibrationBase::dataProcessingMutex_
    std::mutex& sensorDataProcessingMutex_;

    /// Mutex guarding the processing of reference data.
    std::mutex refDataProcessingMutex_;

    /// Subscriber to clicked_point of rviz
    ros::Subscriber clickedPointSubsc_;

    /// Server to provide service to request adding of region marker
    ros::ServiceServer regionMarkerSrv_;

    /// Subscriber to point cloud topic of sensor to be calibrated
    ros::Subscriber srcCloudSubsc_;

    /// Subscriber to point cloud topic of reference model
    ros::Subscriber refCloudSubsc_;

    /// Publisher for regions in source cloud selected by marker and used for calibration
    ros::Publisher srcRegionPub_;

    /// Publisher for regions in ref cloud selected by marker and used for calibration
    ros::Publisher refRegionPub_;

    /// Name of the source LiDAR sensor as given in the URDF model.
    /// This is a reference to ExtrinsicCalibrationBase::srcSensorName_
    std::string& srcLidarSensorName_ =
      ExtrinsicCalibrationBase<LidarDataProcessor, LidarDataProcessor>::srcSensorName_;

    /// Topic name of the source lidar cloud which are to be used for extrinsic calibration.
    /// This is a reference to ExtrinsicCalibrationBase::srcTopicName_
    std::string& srcLidarCloudTopic_ =
      ExtrinsicCalibrationBase<LidarDataProcessor, LidarDataProcessor>::srcTopicName_;

    /// Frame id of source cloud received by #srcCloudSubsc_
    /// This is a reference to ExtrinsicCalibrationBase::srcFrameId_
    std::string& srcCloudFrameId_ =
      ExtrinsicCalibrationBase<LidarDataProcessor, LidarDataProcessor>::srcFrameId_;

    /// Name of the reference
    /// This is a reference to ExtrinsicCalibrationBase::refSensorName_
    std::string& referenceName_ =
      ExtrinsicCalibrationBase<LidarDataProcessor, LidarDataProcessor>::refSensorName_;

    /// Topic name of the reference lidar cloud which are to be used for extrinsic calibration.
    /// This is a reference to ExtrinsicCalibrationBase::refTopicName_
    std::string& refLidarCloudTopic_ =
      ExtrinsicCalibrationBase<LidarDataProcessor, LidarDataProcessor>::refTopicName_;

    /// Frame id of reference cloud received by #refCloudSubsc_
    /// This is a reference to ExtrinsicCalibrationBase::refFrameId_
    std::string& refCloudFrameId_ =
      ExtrinsicCalibrationBase<LidarDataProcessor, LidarDataProcessor>::refFrameId_;

    /// List of 3D points marking region correspondences in the cloud of the source lidar.
    std::vector<InputPointType> srcRegionMarkers_;

    /// List of point clouds holding the seed points for the selected regions in source cloud.
    std::vector<pcl::PointCloud<InputPointType>::Ptr> srcRegionSeedCloudPtrs_;

    /// Pointer to point cloud holding the regions of source sensor used for calibration
    pcl::PointCloud<RegionPointType>::Ptr pSrcRegionsCloud_;

    /// List of 3D points marking region correspondences in the reference data.
    std::vector<InputPointType> refRegionMarkers_;

    /// List of point clouds holding the seed points for the selected regions in reference.
    std::vector<pcl::PointCloud<InputPointType>::Ptr> refRegionSeedCloudPtrs_;

    /// Pointer to point cloud holding the regions of reference data used for calibration
    pcl::PointCloud<RegionPointType>::Ptr pRefRegionsCloud_;

    /// History of region markers added to list. This is used to remove last marker
    std::vector<std::string> regionMarkerHistory_;

    /// Pointer to transform object which will transform the reference Lidar data prior to processing.
    std::shared_ptr<tf::Transform> pRefDataTransform_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_EXTRINSICLIDARLIDARCALIBRATIONNODELET_H