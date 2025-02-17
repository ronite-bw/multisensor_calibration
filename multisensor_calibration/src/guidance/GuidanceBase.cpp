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

#include "../include/multisensor_calibration/guidance/GuidanceBase.h"

// ROS
#include <tf/tf.h>

// multisensor_calibration
#include "../include/multisensor_calibration/common/common.h"
#include "../include/multisensor_calibration/common/utils.hpp"
#include <multisensor_calibration/SensorExtrinsics.h>
namespace multisensor_calibration
{

//==================================================================================================
GuidanceBase::GuidanceBase() :
  isInitialized_(true),
  nodeletName_(""),
  parentNamespace_(""),
  extrinsicSensorPose_(lib3d::Extrinsics()),
  nextTargetPose_(lib3d::Extrinsics()),
  axes_({Eigen::Vector3d(1.0, 0.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0),
         Eigen::Vector3d(0.0, 0.0, 1.0)})
{
}

//==================================================================================================
GuidanceBase::~GuidanceBase()
{
}

//==================================================================================================
float GuidanceBase::computeAxisBound(const Eigen::Vector3d& iPnt, const Eigen::Vector3d& iVec,
                                     const Eigen::Vector4d& iPlane) const
{
    double lambda = (-iPlane(3) - iPlane.head<3>().dot(iPnt)) / (iPlane.head<3>().dot(iVec));

    //--- if lambda is negative, i.e. if it intersects the plane at the back, return infinity
    if (lambda <= 0)
        lambda = INFINITY;

    return static_cast<float>(lambda);
}

//==================================================================================================
bool GuidanceBase::initializeServices(ros::NodeHandle& ioNh)
{
    //--- reset service
    resetSrv_ = ioNh.advertiseService(
      RESET_SRV_NAME,
      &GuidanceBase::onReset, this);

    return true;
}

//==================================================================================================
bool GuidanceBase::initializeSubscribers(ros::NodeHandle& ioNh)
{
    //--- subscriber to calib result
    calibResultSubsc_ = ioNh.subscribe<CalibrationResult_Message_T>(
      calibratorNodeletName_ + "/" + CALIB_RESULT_TOPIC_NAME, 1,
      boost::bind(&GuidanceBase::onCalibrationResultReceived, this, _1));

    return true;
}

//==================================================================================================
bool GuidanceBase::initializeTimers(ros::NodeHandle& ioNh)
{
    //--- initialize trigger to call routine to get calibration meta data
    calibMetaDataTimer_ = ioNh.createTimer(
      ros::Duration(1), &GuidanceBase::getCalibrationMetaData,
      this, false, false);

    return true;
}

//==================================================================================================
bool GuidanceBase::isTargetPoseWithinBoundingPlanes(const lib3d::Extrinsics& iPose) const
{
    bool retVal = true;

    // 4x4 RT matrix of the pose transforming a local point into the reference coordinate system
    const cv::Matx44d POSE_RT = iPose.getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF);

    auto isPntWithinBoundingPlane = [&](const cv::Vec4d& iLocalPnt, const Eigen::Vector4d& iPlane)
    {
        cv::Vec4d refPnt = POSE_RT * iLocalPnt;
        return ((refPnt(0) * iPlane(0) +
                 refPnt(1) * iPlane(1) +
                 refPnt(2) * iPlane(2) +
                 iPlane(3)) > 0);
    };

    for (Eigen::Vector4d plane : fovBoundingPlanes_)
    {
        // Top-Left
        retVal &= isPntWithinBoundingPlane(cv::Vec4d(-calibrationTarget_.boardSize.width / 2,
                                                     calibrationTarget_.boardSize.height / 2,
                                                     0.0,
                                                     1.0),
                                           plane);
        // Top-Right
        retVal &= isPntWithinBoundingPlane(cv::Vec4d(calibrationTarget_.boardSize.width / 2,
                                                     calibrationTarget_.boardSize.height / 2,
                                                     0.0,
                                                     1.0),
                                           plane);
        // Bottom-Right
        retVal &= isPntWithinBoundingPlane(cv::Vec4d(calibrationTarget_.boardSize.width / 2,
                                                     -calibrationTarget_.boardSize.height / 2,
                                                     0.0,
                                                     1.0),
                                           plane);
        // Bottom-Left
        retVal &= isPntWithinBoundingPlane(cv::Vec4d(-calibrationTarget_.boardSize.width / 2,
                                                     -calibrationTarget_.boardSize.height / 2,
                                                     0.0,
                                                     1.0),
                                           plane);
    }

    return retVal;
}

//==================================================================================================
void GuidanceBase::onCalibrationResultReceived(const CalibrationResult_Message_T::ConstPtr& ipResultMsg)
{
    if (!ipResultMsg->isSuccessful)
        return;

    //--- construct transform from pose
    tf::Transform ref2LocalTransform;
    utils::cvtGeometryPose2TfTransform(ipResultMsg->sensorExtrinsics, ref2LocalTransform);

    //--- convert to extrinsic sensor pose
    utils::setCameraExtrinsicsFromTfTransform(ref2LocalTransform,
                                              extrinsicSensorPose_);

    //--- compute overlapping Fov
    computeExtrinsicFovBoundingPlanes();

    //--- compute next target pose
    computeNextTargetPose();
}

//==================================================================================================
void GuidanceBase::onTargetPoseReceived(const TargetBoardPose_Message_T::ConstPtr& ipPoseMsg)
{
    if (ipPoseMsg->isDetection)
    {
        //--- construct transform from pose
        tf::Transform ref2LocalTransform;
        utils::cvtGeometryPose2TfTransform(ipPoseMsg->targetPose, ref2LocalTransform);

        //--- convert to extrinsic sensor pose
        detectedTargetPoses_.push_back(lib3d::Extrinsics());
        utils::setCameraExtrinsicsFromTfTransform(ref2LocalTransform,
                                                  detectedTargetPoses_.back());
    }
}

//==================================================================================================
bool GuidanceBase::readLaunchParameters(const ros::NodeHandle& iNh)
{
    //--- calibrator nodelet name
    calibratorNodeletName_ = iNh.param<std::string>("calibrator_name",
                                                    parentNamespace_ + "/" + CALIBRATOR_SUB_NAMESPACE);

    return true;
}

//==================================================================================================
void GuidanceBase::getCalibrationMetaData(const ros::TimerEvent&)
{
    //--- get calibration meta data
    ros::ServiceClient metaDataClient =
      nh_.serviceClient<CalibrationMetaData>(calibratorNodeletName_ +
                                             "/" + REQUEST_META_DATA_SRV_NAME);
    CalibrationMetaData srvMsg;
    if (!metaDataClient.call(srvMsg))
    {
        ROS_ERROR("[%s] Failed to get calibration meta data. "
                  "Check if calibration nodelet is initialized!",
                  nodeletName_.c_str());
        return;
    }

    calibrationMetaData_ = srvMsg.response;

    //--- if calibration metadata is complete stop timer
    if (calibrationMetaData_.isComplete)
    {
        calibMetaDataTimer_.stop();

        // read calibration target
        calibrationTarget_.readFromYamlFile(calibrationMetaData_.calib_target_file_path);
        isInitialized_ &= calibrationTarget_.isValid();

        // get initial sensor pose
        isInitialized_ &= getInitialSensorPose();

        // initialize subscribers
        isInitialized_ &= initializeSubscribers(nh_);
    }
}

//==================================================================================================
bool GuidanceBase::getInitialSensorPose()
{
    //--- get sensor extrinsics
    ros::ServiceClient extrinsicsClient =
      nh_.serviceClient<SensorExtrinsics>(calibratorNodeletName_ +
                                          "/" + REQUEST_SENSOR_EXTRINSICS_SRV_NAME);
    SensorExtrinsics srvMsg;
    const geometry_msgs::Pose& POSE = srvMsg.response.extrinsics;
    if (!extrinsicsClient.call(srvMsg))
    {
        ROS_ERROR("[%s] Failed to get sensor extrinsics. "
                  "Check if calibration nodelet is initialized!",
                  nodeletName_.c_str());
        return false;
    }

    //--- check if extrinsic pose is 0
    if (tf::Vector3(POSE.position.x,
                    POSE.position.y,
                    POSE.position.z) == tf::Vector3(0, 0, 0) &&
        tf::Quaternion(POSE.orientation.x,
                       POSE.orientation.y,
                       POSE.orientation.z,
                       POSE.orientation.w) == tf::Quaternion(0, 0, 0, 1))
    {
        ROS_INFO("[%s] No initial sensor pose available. Please add first observation:"
                 "\n\t> Place target in field-of-view of both sensors."
                 "\n\t> Press button to add observation.",
                 nodeletName_.c_str());

        return true;
    }
    else
    {
        //--- construct transform from pose
        tf::Transform ref2LocalTransform;
        utils::cvtGeometryPose2TfTransform(POSE, ref2LocalTransform);

        //--- convert to extrinsic sensor pose
        utils::setCameraExtrinsicsFromTfTransform(ref2LocalTransform,
                                                  extrinsicSensorPose_);

        return true;
    }
}

//==================================================================================================
bool GuidanceBase::onReset(multisensor_calibration::ResetCalibration::Request& iReq,
                           multisensor_calibration::ResetCalibration::Response& oRes)
{
    UNUSED_VAR(iReq);

    extrinsicSensorPose_ = lib3d::Extrinsics();
    detectedTargetPoses_.clear();
    resetNextTargetPose();

    oRes.isAccepted = true;
    oRes.msg        = "Guidance is reset.";

    return true;
}

} // namespace multisensor_calibration