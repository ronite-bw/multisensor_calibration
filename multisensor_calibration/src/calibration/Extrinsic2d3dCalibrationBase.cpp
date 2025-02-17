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

#include "multisensor_calibration/calibration/Extrinsic2d3dCalibrationBase.h"

// Std
#include <thread>
#include <vector>

// multisensor_calibration
#include "multisensor_calibration/common/utils.hpp"

namespace multisensor_calibration
{

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  Extrinsic2d3dCalibrationBase(ECalibrationType type) :
  ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>(type),
  cameraInfoTopic_(""),
  imageState_(STR_2_IMG_STATE.at(DEFAULT_IMG_STATE_STR)),
  isStereoCamera_(false),
  rightCameraSensorName_(DEFAULT_CAMERA_SENSOR_NAME),
  rightCameraInfoTopic_(""),
  rightImageFrameId_(""),
  rectSuffix_("_rect"),
  leftCameraInfo_(),
  rightCameraInfo_()
{
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::~Extrinsic2d3dCalibrationBase()
{
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
void Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  calculateAdditionalStereoCalibrations()
{
    calibResult_.calibrations.resize(4);

    tf::Matrix3x3 leftRectMat =
      tf::Matrix3x3(leftCameraInfo_.R[0], leftCameraInfo_.R[1], leftCameraInfo_.R[2],
                    leftCameraInfo_.R[3], leftCameraInfo_.R[4], leftCameraInfo_.R[5],
                    leftCameraInfo_.R[6], leftCameraInfo_.R[7], leftCameraInfo_.R[8]);

    tf::Matrix3x3 rightRectMat =
      tf::Matrix3x3(rightCameraInfo_.R[0], rightCameraInfo_.R[1], rightCameraInfo_.R[2],
                    rightCameraInfo_.R[3], rightCameraInfo_.R[4], rightCameraInfo_.R[5],
                    rightCameraInfo_.R[6], rightCameraInfo_.R[7], rightCameraInfo_.R[8]);

    double roll, pitch, yaw;

    //--- if image state is DISTORTED or UNDISTORTED, calculate position of frame of left
    //--- rectified image
    //--- the rectified frame is at the same position as the distorted frame, but rotated by the
    //--- transposed (inverse) rectification matrix provided in the camera_info
    if (imageState_ == DISTORTED ||
        imageState_ == UNDISTORTED)
    {
        calibResult_.calibrations[1].srcSensorName =
          calibResult_.calibrations[0].srcSensorName + rectSuffix_;
        calibResult_.calibrations[1].srcFrameId =
          calibResult_.calibrations[0].srcFrameId + rectSuffix_;
        calibResult_.calibrations[1].refSensorName =
          calibResult_.calibrations[0].srcSensorName;
        calibResult_.calibrations[1].refFrameId =
          calibResult_.calibrations[0].srcFrameId;
        calibResult_.calibrations[1].baseFrameId = "";

        calibResult_.calibrations[1].XYZ = tf::Vector3(0.f, 0.f, 0.f);

        leftRectMat.transpose().getRPY(roll, pitch, yaw);
        calibResult_.calibrations[1].RPY = tf::Vector3(roll, pitch, yaw);
    }
    //--- if image state is STEREO_RECTIFIED, calculate position of frame of left distorted image
    //--- the distorted frame is at the same position as the rectified frame, but rotated by the
    //--- rectification matrix provided in the camera_info
    else
    {
        //--- copy rectified calibration to second position to preserve order
        calibResult_.calibrations[1] = calibResult_.calibrations[0];

        auto suffixPos = calibResult_.calibrations[0].srcSensorName.rfind(rectSuffix_);
        if (suffixPos != std::string::npos)
            calibResult_.calibrations[0].srcSensorName.replace(
              suffixPos, rectSuffix_.length(), "");

        suffixPos = calibResult_.calibrations[0].srcFrameId.rfind(rectSuffix_);
        if (suffixPos != std::string::npos)
            calibResult_.calibrations[0].srcFrameId.replace(
              suffixPos, rectSuffix_.length(), "");
        calibResult_.calibrations[0].baseFrameId = "";
        calibResult_.calibrations[0].refSensorName =
          calibResult_.calibrations[1].srcSensorName;
        calibResult_.calibrations[0].refFrameId =
          calibResult_.calibrations[1].srcFrameId;

        calibResult_.calibrations[0].XYZ = tf::Vector3(0.f, 0.f, 0.f);

        leftRectMat.getRPY(roll, pitch, yaw);
        calibResult_.calibrations[0].RPY = tf::Vector3(roll, pitch, yaw);
    }

    //--- calculate pose of right rectified camera from left rectified camera
    //--- the pose of the right rectified frame is moved by the baseline of the stereo pair along
    //--- the x axis of the left rectified frame
    //--- the baseline is computed from P(0,3)/-P(0,0) stored inside the right camera info
    calibResult_.calibrations[3].srcSensorName = rightCameraSensorName_;
    calibResult_.calibrations[3].srcFrameId    = rightCameraInfo_.header.frame_id + rectSuffix_;
    if (imageState_ == DISTORTED ||
        imageState_ == UNDISTORTED)
    {
        calibResult_.calibrations[3].srcSensorName += rectSuffix_;
    }
    calibResult_.calibrations[3].refSensorName =
      calibResult_.calibrations[1].srcSensorName;
    calibResult_.calibrations[3].refFrameId =
      calibResult_.calibrations[1].srcFrameId;
    calibResult_.calibrations[3].baseFrameId = "";

    calibResult_.calibrations[3].XYZ =
      tf::Vector3((-rightCameraInfo_.P[3] / rightCameraInfo_.P[0]), 0.f, 0.f);

    calibResult_.calibrations[3].RPY = tf::Vector3(0.f, 0.f, 0.f);

    //--- calculate pose of right undistorted camera from right rectified camera
    //--- the distorted frame is at the same position as the rectified frame, but rotated by the
    //--- rectification matrix provided in the camera_info
    calibResult_.calibrations[2].srcSensorName = rightCameraSensorName_;
    calibResult_.calibrations[2].srcFrameId    = rightCameraInfo_.header.frame_id;
    if (imageState_ == STEREO_RECTIFIED)
    {
        auto suffixPos = calibResult_.calibrations[2].srcSensorName.rfind(rectSuffix_);
        if (suffixPos != std::string::npos)
            calibResult_.calibrations[2].srcSensorName.replace(
              suffixPos, rectSuffix_.length(), "");
    }
    calibResult_.calibrations[2].refSensorName =
      calibResult_.calibrations[3].srcSensorName;
    calibResult_.calibrations[2].refFrameId =
      calibResult_.calibrations[3].srcFrameId;
    calibResult_.calibrations[2].baseFrameId = "";

    calibResult_.calibrations[2].XYZ = tf::Vector3(0.f, 0.f, 0.f);

    rightRectMat.getRPY(roll, pitch, yaw);
    calibResult_.calibrations[2].RPY = tf::Vector3(roll, pitch, yaw);
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
std::pair<double, int> Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::runPnp(
  const std::vector<cv::Point2f>::const_iterator& iCamObsStart,
  const std::vector<cv::Point2f>::const_iterator& iCamObsEnd,
  const std::vector<cv::Point3f>::const_iterator& iLidarObsStart,
  const std::vector<cv::Point3f>::const_iterator& iLidarObsEnd,
  const lib3d::Intrinsics& iCameraIntrinsics,
  const float& iInlierMaxRpjError,
  const bool& iUsePoseGuess,
  lib3d::Extrinsics& oNewSensorExtrinsics,
  const std::vector<uint>& iIndices) const
{
    std::vector<cv::Point3f> lidarCornerObs; // corner observations in lidar data
    std::vector<cv::Point2f> camCornerObs;   // corner observations in camera data

    //--- initialize 3d and 2d corner observations
    //--- if index list is empty copy all points, if not iterate over indices and copy only
    //--- indexed observations
    if (iIndices.empty())
    {
        std::copy(iLidarObsStart, iLidarObsEnd, std::back_inserter(lidarCornerObs));
        std::copy(iCamObsStart, iCamObsEnd, std::back_inserter(camCornerObs));
    }
    else
    {
        const int LIDAR_OBS_END_IDX = std::distance(iCamObsStart, iCamObsEnd);
        const int CAM_OBS_END_IDX   = std::distance(iCamObsStart, iCamObsEnd);
        for (uint idx : iIndices)
        {
            if (idx < static_cast<uint>(LIDAR_OBS_END_IDX))
                lidarCornerObs.push_back(*(iLidarObsStart + idx));
            if (idx < static_cast<uint>(CAM_OBS_END_IDX))
                camCornerObs.push_back(*(iCamObsStart + idx));
        }
    }

    //--- initialize extrinsic guess
    lib3d::Extrinsics prevExtrinsics =
      ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::sensorExtrinsics_.back();
    prevExtrinsics.setTransfDirection(lib3d::Extrinsics::REF_2_LOCAL);
    cv::Vec3d cameraRVec = prevExtrinsics.getRotationVec();
    cv::Vec3d cameraTVec = prevExtrinsics.getTranslationVec();

    //--- run solvePnP
    std::vector<int> inliers;
    cv::solvePnPRansac(lidarCornerObs, camCornerObs,
                       iCameraIntrinsics.getK_as3x3(), iCameraIntrinsics.getDistortionCoeffs(),
                       cameraRVec, cameraTVec, iUsePoseGuess, 100,
                       iInlierMaxRpjError,
                       0.99, inliers);

    //--- calculate reprojection error
    double rpjError = utils::calculateMeanReprojectionError(camCornerObs,
                                                            lidarCornerObs,
                                                            iCameraIntrinsics.getK_as3x3(),
                                                            iCameraIntrinsics.getDistortionCoeffs(),
                                                            cameraRVec, cameraTVec,
                                                            inliers);

    //--- set new extrinsics
    oNewSensorExtrinsics.setTransfDirection(lib3d::Extrinsics::REF_2_LOCAL);
    oNewSensorExtrinsics.setRotationVec(cameraRVec);
    oNewSensorExtrinsics.setTranslationVec(cameraTVec);

    return std::make_pair(rpjError, static_cast<int>(inliers.size()));
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  initializeCameraIntrinsics(CameraDataProcessor* iopCamProcessor)
{
    // pointer to camera info message
    sensor_msgs::CameraInfoConstPtr camInfo =
      ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cameraInfoTopic_,
                                                          ros::Duration(20, 0));
    if (camInfo != nullptr)
    {
        // ROS_INFO("frame_id %s", camInfo->header.frame_id.c_str());

        lib3d::Intrinsics cameraIntr;
        utils::setCameraIntrinsicsFromCameraInfo(*camInfo.get(),
                                                 cameraIntr,
                                                 imageState_);
        iopCamProcessor->setCameraIntrinsics(cameraIntr);

        //--- if camera is to be calibrated as stereo, store camera info in member variable
        //--- and wait for camera info of right camera
        if (isStereoCamera_)
        {
            sensor_msgs::CameraInfoConstPtr rightCamInfo =
              ros::topic::waitForMessage<sensor_msgs::CameraInfo>(rightCameraInfoTopic_,
                                                                  ros::Duration(20, 0));

            if (rightCamInfo != nullptr)
            {
                leftCameraInfo_  = *camInfo;
                rightCameraInfo_ = *rightCamInfo;
            }
            else
            {
                ROS_ERROR("[%s] Wait for message of 'camera_info' topic for right camera has "
                          "timed out. "
                          "\n'camera_info' topic: %s"
                          "\nWaiting for next data package.",
                          CalibrationBase::nodeletName_.c_str(), rightCameraInfoTopic_.c_str());
                return false;
            }
        }
    }
    else
    {
        ROS_ERROR("[%s] Wait for message of 'camera_info' topic has timed out. "
                  "\n'camera_info' topic: %s"
                  "\nWaiting for next data package.",
                  CalibrationBase::nodeletName_.c_str(), cameraInfoTopic_.c_str());
        return false;
    }

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  saveCalibrationSettingsToWorkspace()
{
    if (!ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
          saveCalibrationSettingsToWorkspace())
        return false;

    QSettings* pCalibSettings = CalibrationBase::pCalibrationWs_->settingsPtr();
    if (!pCalibSettings)
        return false;

    //--- camera sensor name
    pCalibSettings->setValue("camera/sensor_name",
                             QString::fromStdString(cameraSensorName_));

    //--- camera image topic
    pCalibSettings->setValue("camera/image_topic",
                             QString::fromStdString(cameraImageTopic_));

    //--- camera info topic
    pCalibSettings->setValue("camera/info_topic",
                             QString::fromStdString(cameraInfoTopic_));

    //--- camera image state
    pCalibSettings->setValue("camera/image_state",
                             QVariant::fromValue(static_cast<int>(imageState_)));

    //--- is stereo camera
    pCalibSettings->setValue("camera/is_stereo_camera",
                             QVariant::fromValue(isStereoCamera_));

    //--- right camera sensor name
    pCalibSettings->setValue("camera/right_sensor_name",
                             QString::fromStdString(rightCameraSensorName_));

    //--- right camera info topic
    pCalibSettings->setValue("camera/right_info_topic",
                             QString::fromStdString(rightCameraInfoTopic_));

    //--- rect frame id suffix
    pCalibSettings->setValue("camera/rect_suffix",
                             QString::fromStdString(rectSuffix_));

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool Extrinsic2d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  readLaunchParameters(const ros::NodeHandle& iNh)
{
    if (!ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
          readLaunchParameters(iNh))
        return false;

    //--- camera_sensor_name
    cameraSensorName_ =
      CalibrationBase::readStringLaunchParameter(iNh, "camera_sensor_name",
                                                 DEFAULT_CAMERA_SENSOR_NAME);

    //--- camera_image_topic
    cameraImageTopic_ =
      CalibrationBase::readStringLaunchParameter(iNh, "camera_image_topic",
                                                 DEFAULT_CAMERA_IMAGE_TOPIC);

    //--- camera_info_topic
    cameraInfoTopic_ = iNh.param<std::string>("camera_info_topic", "");
    if (cameraInfoTopic_.empty())
    {
        cameraInfoTopic_ =
          cameraImageTopic_.substr(0, cameraImageTopic_.find_last_of('/')) + "/camera_info";
    }
    // ROS_INFO("cam info %s", cameraInfoTopic_.c_str());

    //--- image state
    std::string imageStateStr;
    iNh.param<std::string>("image_state", imageStateStr, DEFAULT_IMG_STATE_STR);
    auto findItr = STR_2_IMG_STATE.find(imageStateStr);
    if (findItr != STR_2_IMG_STATE.end())
        imageState_ = findItr->second;
    else
        ROS_WARN("[%s]"
                 "\n\t> String passed to 'image_state' is not valid. "
                 "\n\t> Setting 'image_state' to default: %s",
                 CalibrationBase::nodeletName_.c_str(),
                 DEFAULT_IMG_STATE_STR.c_str());

    //--- is_stereo_camera
    iNh.param<bool>("is_stereo_camera", isStereoCamera_, false);

    //--- right_camera_sensor_name
    rightCameraSensorName_ = iNh.param<std::string>("right_camera_sensor_name", "");

    //--- right_camera_info_topic
    rightCameraInfoTopic_ = iNh.param<std::string>("right_camera_info_topic", "");

    //--- rect_suffix
    rectSuffix_ = CalibrationBase::readStringLaunchParameter(iNh, "rect_suffix",
                                                             "_rect");

    return true;
}

template class Extrinsic2d3dCalibrationBase<CameraDataProcessor, LidarDataProcessor>;
template class Extrinsic2d3dCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>;

} // namespace multisensor_calibration