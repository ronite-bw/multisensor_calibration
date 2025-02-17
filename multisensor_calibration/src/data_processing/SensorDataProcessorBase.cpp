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

#include "../include/multisensor_calibration/data_processing/SensorDataProcessorBase.h"

// Std
#include <cmath>

// Eigen
#include <Eigen/Geometry>

// ROS
#include <tf/tf.h>

// OpenCV
#include <opencv2/core/eigen.hpp>

// multisensor_calibration
#include "../include/multisensor_calibration/common/common.h"
#include "../include/multisensor_calibration/common/utils.hpp"

namespace multisensor_calibration
{

//==================================================================================================
template <class SensorDataT>
SensorDataProcessorBase<SensorDataT>::SensorDataProcessorBase(
  const std::string& iNodeletName,
  const std::string& iSensorName,
  const fs::path& iCalibTargetFilePath) :
  nodeletName_(iNodeletName),
  sensorName_(iSensorName),
  isInitialized_(false),
  calibrationTarget_(CalibrationTarget()),
  markerIdRange_(std::make_pair<int, int>(INT_MAX, INT_MIN)),
  capturedCalibTargetPoses_()
{
    calibrationTarget_.readFromYamlFile(iCalibTargetFilePath);
    if (!calibrationTarget_.isValid())
    {
        ROS_FATAL("[%s]"
                  "\n\t> Calibration target is not valid!"
                  "\n\t> Target configuration file: %s",
                  nodeletName_.c_str(), iCalibTargetFilePath.string().c_str());
        return;
    }

    for (int id : calibrationTarget_.markerIds)
    {
        markerIdRange_.first  = std::min(markerIdRange_.first, id);
        markerIdRange_.second = std::max(markerIdRange_.second, id);
    }

    isInitialized_ = true;
}

//==================================================================================================
template <class SensorDataT>
SensorDataProcessorBase<SensorDataT>::~SensorDataProcessorBase()
{
}

//==================================================================================================
template <class SensorDataT>
std::vector<lib3d::Extrinsics> SensorDataProcessorBase<SensorDataT>::
  getCalibrationTargetPoses() const
{
    return capturedCalibTargetPoses_;
}

//==================================================================================================
template <class SensorDataT>
lib3d::Extrinsics SensorDataProcessorBase<SensorDataT>::getLastCalibrationTargetPose() const
{
    return capturedCalibTargetPoses_.back();
}

//==================================================================================================
template <class SensorDataT>
uint SensorDataProcessorBase<SensorDataT>::getNumCalibIterations() const
{
    return capturedCalibTargetPoses_.size();
}

//==================================================================================================
template <class SensorDataT>
std::string SensorDataProcessorBase<SensorDataT>::getSensorName() const
{
    return sensorName_;
}

//==================================================================================================
template <class SensorDataT>
void SensorDataProcessorBase<SensorDataT>::publishLastCalibrationTargetPose(
  const std_msgs::Header& iHeader,
  const lib3d::Extrinsics& iPose,
  const bool& iIsDetection,
  const ros::Publisher& iPub) const
{
    //--- Publish multi-array with up vector and center of detected target

    // ROS message holding upd vector and center of detected target board
    TargetBoardPose_Message_T poseMsg;
    poseMsg.header      = iHeader;
    poseMsg.isDetection = iIsDetection;

    tf::Transform poseTransform;
    utils::setTfTransformFromCameraExtrinsics(iPose, poseTransform);
    utils::cvtTfTransform2GeometryPose(poseTransform, poseMsg.targetPose);

    iPub.publish(poseMsg);

    ROS_INFO("[%s] Published pose of detected calibration target!", getLoggingId().c_str());
}

//==================================================================================================
template <class SensorDataT>
void SensorDataProcessorBase<SensorDataT>::publishPreview(
  const ros::Time& iStamp, std::string& iFrameId) const
{
    //--- construct header
    std_msgs::Header header;
    header.stamp    = iStamp;
    header.frame_id = iFrameId;

    //--- call overloaded method
    publishPreview(header);
}

//==================================================================================================
template <class SensorDataT>
void SensorDataProcessorBase<SensorDataT>::publishLastTargetDetection(
  const ros::Time& iStamp, std::string& iFrameId) const
{
    //--- construct header
    std_msgs::Header header;
    header.stamp    = iStamp;
    header.frame_id = iFrameId;

    //--- call overloaded method
    publishLastTargetDetection(header);
}

//==================================================================================================
template <class SensorDataT>
bool SensorDataProcessorBase<SensorDataT>::removeCalibIteration(
  const uint& iIterationId)
{
    if (iIterationId > this->getNumCalibIterations())
        return false;

    capturedCalibTargetPoses_.erase(capturedCalibTargetPoses_.begin() + (iIterationId - 1));

    return true;
}

//==================================================================================================
template <class SensorDataT>
bool SensorDataProcessorBase<SensorDataT>::saveObservations(fs::path iOutputPath) const
{
    bool isSuccessful = true;

    //--- loop through calibration iterations
    for (uint calibItr = 0; calibItr < capturedCalibTargetPoses_.size(); ++calibItr)
    {

        try
        {
            // directory of each calibration iteration
            fs::path iterationOutputPath = iOutputPath;
            iterationOutputPath.append(std::to_string(calibItr + 1));

            if (!fs::exists(iterationOutputPath))
                fs::create_directories(iterationOutputPath);

            //--- store target pose
            tf::Transform refToSrcTransform;
            utils::setTfTransformFromCameraExtrinsics(capturedCalibTargetPoses_[calibItr],
                                                      refToSrcTransform);
            auto xyz = refToSrcTransform.inverse().getOrigin(); // invert to get LOCAL_2_REF
            double roll, pitch, yaw;
            refToSrcTransform.inverse().getBasis().getRPY(roll, pitch, yaw); // invert to get LOCAL_2_REF

            std::fstream targetPoseFout(
              iterationOutputPath.string() + "/" + sensorName_ + "_target_pose.txt",
              std::ios_base::out);
            targetPoseFout << "# Pose of the detected calibration target with respect to sensor origin." << std::endl;
            targetPoseFout << "#    XYZ: Translation in x-, y-, and z-direction." << std::endl;
            targetPoseFout << "#    RPY: Orientation in the form of roll, pitch, and yaw in radian." << std::endl;
            targetPoseFout << "XYZ: " << xyz.x() << " " << xyz.y() << " " << xyz.z() << std::endl;
            targetPoseFout << "RPY: " << roll << " " << pitch << " " << yaw << std::endl;
            targetPoseFout.close();
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("[%s] %s", nodeletName_.c_str(), e.what());
            isSuccessful = false;
        }
    }

    return isSuccessful;
}

//==================================================================================================
template <class SensorDataT>
bool SensorDataProcessorBase<SensorDataT>::averageObservations(
  const std::vector<lib3d::Extrinsics>& iCalibTargetPoses,
  lib3d::Extrinsics& oAvgdCalibTargetPose) const
{
    //--- return false if no observations are available
    if (iCalibTargetPoses.empty())
        return false;

    //--- if only one pose is available, directly return stored observations
    if (iCalibTargetPoses.size() == 1)
    {
        oAvgdCalibTargetPose = iCalibTargetPoses.front();

        return true;
    }

    // number of poses
    const int N_POSES = static_cast<int>(iCalibTargetPoses.size());

    // 1 / (number of pose), used for averaging weight
    const float ONE_OVER_N_POSES = (1.f / static_cast<float>(N_POSES));

    // translation of the averaged pose
    cv::Vec3d averagedTranslation(0, 0, 0);

    // list of quaternion corresponding to rotations of the poses
    std::vector<Eigen::Vector4f> rotationQuaternions;

    //--- loop over pose
    //--- aggregate and average translation into output variable
    //--- populate list of quaternions
    for (lib3d::Extrinsics pose : iCalibTargetPoses)
    {
        //--- set correct transformation direction
        pose.setTransfDirection(lib3d::Extrinsics::LOCAL_2_REF);

        //--- aggregate / average translation
        averagedTranslation += (ONE_OVER_N_POSES * pose.getTranslationVec());

        //--- get quaternion from rotation and store in list to average
        Eigen::Matrix3d eigenRotMat; // rotation matrix in Eigen
        cv::cv2eigen(pose.getRotationMat(), eigenRotMat);
        Eigen::Quaterniond eigenRotQuat(eigenRotMat); // rotation as quaternion
        rotationQuaternions.push_back(Eigen::Vector4f(static_cast<float>(eigenRotQuat.x()),
                                                      static_cast<float>(eigenRotQuat.y()),
                                                      static_cast<float>(eigenRotQuat.z()),
                                                      static_cast<float>(eigenRotQuat.w())));
    }

    //--- average quaternions
    Eigen::Vector4f averagedRotationQuat = utils::averageQuaternion(rotationQuaternions);
    cv::Mat averagedRotation;
    cv::eigen2cv(Eigen::Quaterniond(static_cast<double>(averagedRotationQuat.w()),
                                    static_cast<double>(averagedRotationQuat.x()),
                                    static_cast<double>(averagedRotationQuat.y()),
                                    static_cast<double>(averagedRotationQuat.z()))
                   .normalized()
                   .toRotationMatrix(),
                 averagedRotation);

    //--- store in output variable, set local to ref to handle absolute translation and rotation
    oAvgdCalibTargetPose = lib3d::Extrinsics(lib3d::Extrinsics::LOCAL_2_REF);
    oAvgdCalibTargetPose.setTranslationVec(averagedTranslation);
    oAvgdCalibTargetPose.setRotationMat(averagedRotation);

    return true;
}

//==================================================================================================
template <class SensorDataT>
std::string SensorDataProcessorBase<SensorDataT>::getLoggingId() const
{
    std::string id = (sensorName_.empty())
                       ? nodeletName_
                       : nodeletName_ + "/" + sensorName_;

    return id;
}

//==================================================================================================
template <class SensorDataT>
void SensorDataProcessorBase<SensorDataT>::reset()
{
    capturedCalibTargetPoses_.clear();
}

} // namespace multisensor_calibration

template class multisensor_calibration::SensorDataProcessorBase<cv::Mat>;
template class multisensor_calibration::SensorDataProcessorBase<
  pcl::PointCloud<multisensor_calibration::InputPointType>>;