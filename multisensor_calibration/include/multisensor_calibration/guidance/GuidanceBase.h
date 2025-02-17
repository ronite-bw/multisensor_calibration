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

#ifndef MULTISENSORCALIBRATION_GUIDANCEBASE_H
#define MULTISENSORCALIBRATION_GUIDANCEBASE_H

// Std
#include <memory>
#include <tuple>

// ROS
#include <ros/ros.h>
#include <ros/service.h>
#include <ros/timer.h>

// Eigen
#include <Eigen/Geometry>

// multisensor_calibration
#include "../common/common.h"
#include "../common/lib3D/core/extrinsics.hpp"
#include "../config/CalibrationTarget.hpp"
#include <multisensor_calibration/CalibrationMetaData.h>
#include <multisensor_calibration/ResetCalibration.h>

namespace multisensor_calibration
{

/**
 * @ingroup guidance
 * @brief Base class for all classes that guide the user in the calibration process.
 *
 * @note The guidance feature of the calibration is currently still in development and not yet
 * functional.
 *
 */
class GuidanceBase
{

    //--- METHOD DECLARATION ---//
  public:
    /**
     * @brief Default constructor.
     */
    GuidanceBase();

    /**
     * @brief Destructor
     */
    virtual ~GuidanceBase();

  protected:
    /**
     * @brief Function to compute bound on axis along 'iVec' w.r.t to 'iPnt' by intersecting the axis
     * with the plane 'iPlane'.
     */
    float computeAxisBound(const Eigen::Vector3d& iPnt, const Eigen::Vector3d& iVec,
                           const Eigen::Vector4d& iPlane) const;

    /**
     * @brief Method to compute overlapping Fov between the two sensors based on the estimated extrinsic
     * pose.
     *
     * @note This is a pure virtual method and needs to be implemented by the specific subclasses.
     */
    virtual void computeExtrinsicFovBoundingPlanes() = 0;

    /**
     * @brief Method to compute Fov of the two sensors based on the intrinsic parameters.
     *
     * @note This is a pure virtual method and needs to be implemented by the specific subclasses.
     */
    virtual bool computeIntrinsicFovBoundingPlanes() = 0;

    /**
     * @brief Method to compute next target pose.
     *
     * @note This is a pure virtual method and needs to be implemented by the specific subclasses.
     */
    virtual void computeNextTargetPose() = 0;

    /**
     * @brief Method to initialize publishers
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    virtual bool initializePublishers(ros::NodeHandle& ioNh) = 0;

    /**
     * @brief Method to initialize services
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    virtual bool initializeServices(ros::NodeHandle& ioNh);

    /**
     * @brief Method to initialize subscribers.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    virtual bool initializeSubscribers(ros::NodeHandle& ioNh);

    /**
     * @brief Method to initialize timers.
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    virtual bool initializeTimers(ros::NodeHandle& ioNh);

    /**
     * @brief Function to check if pose of the calibration target is within the bounding planes.
     *
     * @return True, if target with given pose is within bounding planes. False, otherwise.
     */
    bool isTargetPoseWithinBoundingPlanes(const lib3d::Extrinsics& iPose) const;

    /**
     * @brief Callback function handling calibration results messages.
     *
     * @param[in] ipResultMsg Calibration result message.
     */
    virtual void onCalibrationResultReceived(const CalibrationResult_Message_T::ConstPtr& ipResultMsg);

    /**
     * @brief Callback function handling target pose messages.
     *
     * @param[in] ipPoseMsg Target pose message.
     */
    virtual void onTargetPoseReceived(const TargetBoardPose_Message_T::ConstPtr& ipPoseMsg);

    /**
     * @brief Method to read launch parameters
     *
     * @param[in] iNh Object of node handle
     * @return True if successful. False, otherwise (e.g. if sanity check fails)
     */
    virtual bool readLaunchParameters(const ros::NodeHandle& iNh);

    /**
     * @brief Method to reset nextTargetPose_
     */
    virtual void resetNextTargetPose() = 0;

  private:
    /**
     * @brief Method to get calibration meta data. This is connected to the calibMetaDataTimer_.
     */
    void getCalibrationMetaData(const ros::TimerEvent&);

    /**
     * @brief Method to get initial sensor pose from the calibration object.
     */
    bool getInitialSensorPose();

    /**
     * @brief Service call to request reset of calibration
     *
     * @param[in] iReq Request, empty
     * @param[out] oRes Response.
     */
    bool onReset(multisensor_calibration::ResetCalibration::Request& iReq,
                 multisensor_calibration::ResetCalibration::Response& oRes);

    //--- MEMBER DECLARATION ---//

  protected:
    /// Flag indicating if nodelet is initialized.
    bool isInitialized_;

    /// Name of nodelet.
    std::string nodeletName_;

    /// Name of the parent namespace.
    std::string parentNamespace_;

    /// Global node handler.
    ros::NodeHandle nh_;

    /// Private node handler.
    ros::NodeHandle pnh_;

    /// Name of the calibrator nodelet
    std::string calibratorNodeletName_;

    /// Timer object to trigger service call to get calibration meta data.
    ros::Timer calibMetaDataTimer_;

    /// Subscriber to calibration result messages.
    ros::Subscriber calibResultSubsc_;

    /// Subscriber to target pose messages.
    ros::Subscriber targetPoseSubsc_;

    /// Server to provide service to reset calibration
    ros::ServiceServer resetSrv_;

    /// Member variable holding calibration meta data.
    multisensor_calibration::CalibrationMetaDataResponse calibrationMetaData_;

    /// Extrinsic pose between the sensors to calibrate
    lib3d::Extrinsics extrinsicSensorPose_;

    /// List of planes encapsulating the overlapping FoV between the sensors.
    std::vector<Eigen::Vector4d> fovBoundingPlanes_;

    /// Object of the calibration target used.
    CalibrationTarget calibrationTarget_;

    /// List of target poses detected during calibration
    std::vector<lib3d::Extrinsics> detectedTargetPoses_;

    /// Pose of the calibration target that is to be placed next.
    lib3d::Extrinsics nextTargetPose_;

    /// Axes to be covered by the calibration
    std::array<Eigen::Vector3d, 3> axes_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_GUIDANCEBASE_H