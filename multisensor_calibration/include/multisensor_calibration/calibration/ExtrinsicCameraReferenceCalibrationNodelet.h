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

#ifndef MULTISENSORCALIBRATION_EXTRINSICCAMERAREFERENCECALIBRATIONNODELET_H
#define MULTISENSORCALIBRATION_EXTRINSICCAMERAREFERENCECALIBRATIONNODELET_H

// Std
#include <memory>
#include <string>
#include <tuple>

// ROS
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>

// PCL
#include <pcl/filters/frustum_culling.h>

// multisensor_calibration
#include "../common/common.h"
#include "../data_processing/CameraDataProcessor.h"
#include "../data_processing/ReferenceDataProcessor3d.h"
#include "Extrinsic2d3dCalibrationBase.h"
#include <multisensor_calibration/CameraIntrinsics.h>
#include <multisensor_calibration/ExtrinsicCameraReferenceCalibrationConfig.h>

namespace multisensor_calibration
{

/**
 * @ingroup nodelets
 * @ingroup calibration
 * @brief Nodelet to perform extrinsic camera-reference calibration.
 *
 * This subclasses multisensor_calibration::Extrinsic2d3dCalibrationBase.
 */
class ExtrinsicCameraReferenceCalibrationNodelet
  : public Extrinsic2d3dCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>,
    public nodelet::Nodelet
{

    //--- METHOD DECLARATION ---/
  public:
    /**
     * @brief Default constructor
     */
    ExtrinsicCameraReferenceCalibrationNodelet();

    /**
     * @brief Destructor
     */
    virtual ~ExtrinsicCameraReferenceCalibrationNodelet();

  private:
    /**
     * @brief Callback function handling parameter changes by dynamic reconfigure.
     *
     * @param[in] iConfig Config object
     * @param[in] level
     */
    void dynReconfigureCallback(ExtrinsicCameraReferenceCalibrationConfig& iConfig, uint32_t level);

    /**
     * @brief Method to finalize calibration. This overrides the method of the parent class.
     *
     * This will calibrate the extrinsic pose based on all observations in the list and print the
     * final error and print out the result of the calibration.
     */
    void finalizeCalibration() override;

    /**
     * @brief Method to initialize data processing. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * This will initialize the data processors, call the initialization of the publishers within
     * the data processors and subscribe to the corresponding data topics.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    bool initializeDataProcessors() override;

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
     * @brief Handle service call to get camera intrinsics.
     *
     * @param[in] iReq Request, UNUSED.
     * @param[out] oRes Response.
     */
    bool onRequestCameraIntrinsics(
      multisensor_calibration::CameraIntrinsics::Request& iReq,
      multisensor_calibration::CameraIntrinsics::Response& oRes);

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
     * @brief Method to receive sensor data, i.e. camera image from the source sensor.
     *
     * This calls the processing of the data data processors, i.e. detect the calibration target
     * or possible candidates, depending on wether the command to capture the target is triggered or
     * not. When the target is detected a calibration with the last observation is performed. In
     * this, the observations might be rejected if the error of the calibration is too large.
     *
     * @param[in] ipImgMsg Pointer to vis image message.
     */
    void onSensorDataReceived(const InputImage_Message_T::ConstPtr& ipImgMsg);

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
    dynamic_reconfigure::Server<ExtrinsicCameraReferenceCalibrationConfig> dynConfigServer_;

    /// Dynamic reconfigure object
    ExtrinsicCameraReferenceCalibrationConfig dynConfig_;

    /// Node handle
    using CalibrationBase::nh_;

    /// Server to provide service to get camera intrinsics
    ros::ServiceServer cameraIntrSrv_;

    /// Subscriber to vis image topic
    image_transport::Subscriber imageSubsc_;

    /// Name of the reference data.
    /// This is a reference to ExtrinsicCalibrationBase::refSensorName_
    std::string& referenceName_ =
      ExtrinsicCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>::refSensorName_;

    /// Frame id of reference cloud received by #refCloudSubsc_
    /// This is a reference to ExtrinsicCalibrationBase::refFrameId_
    using ExtrinsicCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>::refFrameId_;

    /// Pointer to camera data processor, responsible to detect calibration target
    /// in camera image data. This is a reference of ExtrinsicCalibrationBase::pSrcDataProcessor_.
    std::shared_ptr<CameraDataProcessor>& pCamDataProcessor_ =
      ExtrinsicCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>::pSrcDataProcessor_;

    /// Pointer to object of reference data processor, responsible to provide calibration target
    /// in the reference data.
    using ExtrinsicCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>::
      pRefDataProcessor_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_EXTRINSICCAMERAREFERENCECALIBRATIONNODELET_H