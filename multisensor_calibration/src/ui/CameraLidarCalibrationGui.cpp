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

#include "../include/multisensor_calibration/ui/CameraLidarCalibrationGui.h"

// Std
#include <future>
#include <string>
#include <thread>

// ROS
#include <tf/tf.h>

// Qt
#include <QCoreApplication>
#include <QMessageBox>
#include <QObject>

// multisensor_calibration
#include "../../include/multisensor_calibration/common/common.h"
#include "../../include/multisensor_calibration/common/utils.hpp"
#include <multisensor_calibration/CameraIntrinsics.h>
#include <multisensor_calibration/SensorExtrinsics.h>
namespace multisensor_calibration
{

//==================================================================================================
CameraLidarCalibrationGui::CameraLidarCalibrationGui(const std::string& iAppTitle,
                                                     const std::string& iGuiSubNamespace) :
  CalibrationGuiBase(iAppTitle, iGuiSubNamespace),
  pPlacementGuidanceDialog_(nullptr),
  pCameraTargetDialog_(nullptr),
  pLidarTargetDialog_(nullptr),
  pFusionDialog_(nullptr)
{
}

//==================================================================================================
CameraLidarCalibrationGui::~CameraLidarCalibrationGui()
{
    //--- unload visualization nodelet if loaded
    pNodeletLoader_->unload(visualizerNodeletName_);
}

//==================================================================================================
void CameraLidarCalibrationGui::initializeGuiContents()
{
    CalibrationGuiBase::initializeGuiContents();

    //--- initialize content of placement guidance dialog
    if (pPlacementGuidanceDialog_)
    {
        pPlacementGuidanceDialog_->subscribeToImageTopic(nh_,
                                                         guidanceNodeletName_ +
                                                           "/" + PLACEMENT_GUIDANCE_TOPIC_NAME);
    }

    //--- initialize content of camera target dialog
    if (pCameraTargetDialog_)
    {
        pCameraTargetDialog_->setWindowTitle(
          QString::fromStdString(calibrationMetaData_.src_sensor_name));

        pCameraTargetDialog_->subscribeToImageTopic(nh_,
                                                    calibratorNodeletName_ +
                                                      "/" + calibrationMetaData_.src_sensor_name +
                                                      "/" + ANNOTATED_CAMERA_IMAGE_TOPIC_NAME);
    }

    //--- initialize content of lidar target visualization
    if (pLidarTargetDialog_)
    {
        pLidarTargetDialog_->setWindowTitle(
          QString::fromStdString(calibrationMetaData_.ref_sensor_name));

        pLidarTargetDialog_->setFixedReferenceFrame((calibrationMetaData_.base_frame_id.empty())
                                                      ? calibrationMetaData_.ref_frame_id
                                                      : calibrationMetaData_.base_frame_id);
        pLidarTargetDialog_->addAxes();
        pLidarTargetDialog_->addRawSensorCloud(calibrationMetaData_.ref_topic_name);
        pLidarTargetDialog_
          ->addRegionsOfInterestCloud(calibratorNodeletName_ +
                                      "/" + calibrationMetaData_.ref_sensor_name +
                                      "/" + ROIS_CLOUD_TOPIC_NAME);
        pLidarTargetDialog_
          ->addCalibTargetCloud(calibratorNodeletName_ +
                                "/" + calibrationMetaData_.ref_sensor_name +
                                "/" + TARGET_PATTERN_CLOUD_TOPIC_NAME);
        pLidarTargetDialog_
          ->addMarkerCornersCloud(calibratorNodeletName_ +
                                  "/" + calibrationMetaData_.ref_sensor_name +
                                  "/" + MARKER_CORNERS_TOPIC_NAME);
    }

    //--- hide progress dialog
    hideProgressDialog();
}

//==================================================================================================
void CameraLidarCalibrationGui::loadVisualizer()
{
    // Function to initialize visualizer nodelet and dialog
    auto initializeAndRunSensorFusion = [&]() -> bool
    {
        //--- get camera intrinsics to extract image state
        ros::ServiceClient intrinsicsClient =
          nh_.serviceClient<CameraIntrinsics>(calibratorNodeletName_ +
                                              "/" + REQUEST_CAM_INTRINSICS_SRV_NAME);
        CameraIntrinsics intrinsicsSrvMsg;
        if (!intrinsicsClient.call(intrinsicsSrvMsg))
        {
            ROS_ERROR("[%s] Failed to get camera intrinsics. "
                      "Check if calibration nodelet is initialized!",
                      guiNodeName_.c_str());

            return false;
        }

        //--- get sensor extrinsics
        ros::ServiceClient extrinsicsClient =
          nh_.serviceClient<SensorExtrinsics>(calibratorNodeletName_ +
                                              "/" + REQUEST_SENSOR_EXTRINSICS_SRV_NAME);
        SensorExtrinsics extrinsicsSrvMsg;
        extrinsicsSrvMsg.request.extrinsic_type = SensorExtrinsics::Request::SENSOR_2_SENSOR;
        geometry_msgs::Pose& extrinsicPose      = extrinsicsSrvMsg.response.extrinsics;
        if (!extrinsicsClient.call(extrinsicsSrvMsg))
        {
            ROS_ERROR("[%s] Failed to get sensor extrinsics. "
                      "Check if calibration nodelet is initialized!",
                      guiNodeName_.c_str());

            return false;
        }

        //--- check if extrinsic pose is 0
        if (tf::Vector3(extrinsicPose.position.x,
                        extrinsicPose.position.y,
                        extrinsicPose.position.z) == tf::Vector3(0, 0, 0) &&
            tf::Quaternion(extrinsicPose.orientation.x,
                           extrinsicPose.orientation.y,
                           extrinsicPose.orientation.z,
                           extrinsicPose.orientation.w) == tf::Quaternion(0, 0, 0, 1))
        {
            ROS_ERROR("[%s] Cannot open calibration visualization. "
                      "No extrinsic sensor pose available.",
                      guiNodeName_.c_str());

            return false;
        }

        //--- construct transform from pose
        tf::Transform ref2LocalTransform;
        utils::cvtGeometryPose2TfTransform(extrinsicPose, ref2LocalTransform);
        tf::Transform local2RefTransform = ref2LocalTransform.inverse(); // invert to get absolute position and rotation
        double roll, pitch, yaw;
        local2RefTransform.getBasis().getRPY(roll, pitch, yaw);

        //--- construct temporary transform string
        std::stringstream temporaryTransform;
        temporaryTransform.imbue(std::locale("en_US.UTF-8"));
        temporaryTransform << local2RefTransform.getOrigin().x() << " "
                           << local2RefTransform.getOrigin().y() << " "
                           << local2RefTransform.getOrigin().z() << " "
                           << roll << " "
                           << pitch << " "
                           << yaw;

        std::cout << temporaryTransform.str() << std::endl;

        //--- load nodelet
        nodelet::V_string args;
        nodelet::M_string remappings = {
          {"pointcloud", calibrationMetaData_.ref_topic_name},
          {"image", calibrationMetaData_.src_topic_name},
        };
        ros::param::set(visualizerNodeletName_ + "/image_state", intrinsicsSrvMsg.response.image_state);
        ros::param::set(visualizerNodeletName_ + "/min_depth", 0.5);
        ros::param::set(visualizerNodeletName_ + "/max_depth", 25);
        ros::param::set(visualizerNodeletName_ + "/temp_transform", temporaryTransform.str());
        pNodeletLoader_->load(visualizerNodeletName_,
                              "multisensor_calibration/PointCloud2ImageNodelet", remappings, args);

        return true;
    };

    //--- show progress dialog
    showProgressDialog("Initializing visualizer nodelet ...");

    //--- initialize visualizer asynchronously
    auto initFuture = std::async(initializeAndRunSensorFusion);

    //--- while future is not ready, process QEvents in order for the progress dialog not to freeze
    while (initFuture.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
        QCoreApplication::processEvents();

    if (initFuture.get() == true)
    {
        //--- load dialog
        if (pFusionDialog_ == nullptr)
        {
            pFusionDialog_ = std::make_shared<ImageViewDialog>(pCalibControlWindow_.get());
            pFusionDialog_->setWindowModality(Qt::NonModal);
            pFusionDialog_->setWindowTitle("Sensor Fusion");
            pFusionDialog_->subscribeToImageTopic(nh_, visualizerNodeletName_ +
                                                         "/fused_image");

            //--- connect rejection signal, i.e. close signal of dialog, this will unload nodelet when dialog is closed
            QObject::connect(pFusionDialog_.get(), &QDialog::rejected,
                             [=]()
                             {
                                 pNodeletLoader_->unload(visualizerNodeletName_);

                                 pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled(true);
                                 pCalibControlWindow_->pbVisCalibrationPtr()->setChecked(false);
                             });
        }

        pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled(false);
        pCalibControlWindow_->pbVisCalibrationPtr()->setChecked(true);
        pFusionDialog_->show();

        QMessageBox::information(pFusionDialog_.get(), pFusionDialog_->windowTitle(),
                                 QObject::tr(
                                   "In order to visualize the calibration, the 3D points of the "
                                   "LiDAR sensor are projected into the camera image and colorized "
                                   "according to their distance from the camera. Thus, if the "
                                   "calibration is good, then the structure in the LiDAR scan "
                                   "(which can be derived from the depth coloring) should align "
                                   "with the object in the image."));
    }
    else
    {
        pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled(true);
        pCalibControlWindow_->pbVisCalibrationPtr()->setChecked(false);
    }

    hideProgressDialog();
}

//==================================================================================================
bool CameraLidarCalibrationGui::setupGuiElements()
{
    bool isSuccessful = CalibrationGuiBase::setupGuiElements();
    if (!isSuccessful)
        return false;

    pCalibControlWindow_->setWindowTitle(
      QString::fromStdString(CALIB_TYPE_2_STR.at(EXTRINSIC_CAMERA_LIDAR_CALIBRATION)) + " Calibration");

    //--- setup placement guidance dialog at the top-right corner of the display
    pPlacementGuidanceDialog_ = std::make_shared<ImageViewDialog>(pCalibControlWindow_.get());
    if (!pPlacementGuidanceDialog_)
        return false;
    pPlacementGuidanceDialog_->setWindowTitle("Target Placement Guidance");
    pPlacementGuidanceDialog_->move(screenGeometry_.width() / 2, 0);
    pPlacementGuidanceDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                            (screenGeometry_.height() / 2) - titleBarHeight_ - 1);
    pCalibControlWindow_->attachPlacementGuidanceDialog(pPlacementGuidanceDialog_.get());
    pPlacementGuidanceDialog_->show();

    //--- setup camera target dialog at the bottom-left corner of the display
    pCameraTargetDialog_ = std::make_shared<ImageViewDialog>(pCalibControlWindow_.get());
    if (!pCameraTargetDialog_)
        return false;
    pCameraTargetDialog_->setWindowTitle("Camera Target Detections");
    pCameraTargetDialog_->move(0, (screenGeometry_.height() / 2) + (2 * titleBarHeight_));
    pCameraTargetDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                       (screenGeometry_.height() / 2) - titleBarHeight_ - 1);

    pCalibControlWindow_->attachSourceDialog(pCameraTargetDialog_.get());
    pCameraTargetDialog_->show();

    //--- setup lidar target dialog at the bottom-right corner of the display
    pLidarTargetDialog_ = std::make_shared<Rviz3dViewDialog>(pCalibControlWindow_.get());
    if (!pLidarTargetDialog_)
        return false;
    pLidarTargetDialog_->setWindowTitle("LiDAR Target Detections");
    pLidarTargetDialog_->move(screenGeometry_.width() / 2,
                              (screenGeometry_.height() / 2) + (2 * titleBarHeight_));
    pLidarTargetDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                      (screenGeometry_.height() / 2) - titleBarHeight_ - 1);
    pCalibControlWindow_->attachReferenceDialog(pLidarTargetDialog_.get());
    pLidarTargetDialog_->show();

    //--- show infinite progress dialog at screen center
    showProgressDialog("Initializing user interface ...");

    return true;
}

} // namespace multisensor_calibration
