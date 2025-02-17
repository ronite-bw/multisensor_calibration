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

#include "../include/multisensor_calibration/ui/LidarReferenceCalibrationGui.h"

// Std
#include <future>
#include <string>
#include <thread>

// Qt
#include <QCoreApplication>
#include <QLabel>
#include <QMessageBox>
#include <QObject>

// ROS
#include <tf/tf.h>

// multisensor_calibration
#include "../../include/multisensor_calibration/common/utils.hpp"
#include "../include/multisensor_calibration/common/common.h"
#include <multisensor_calibration/SensorExtrinsics.h>

namespace multisensor_calibration
{

//==================================================================================================
LidarReferenceCalibrationGui::LidarReferenceCalibrationGui(const std::string& iAppTitle,
                                                           const std::string& iGuiSubNamespace) :
  CalibrationGuiBase(iAppTitle, iGuiSubNamespace),
  pPlacementGuidanceDialog_(nullptr),
  pSrcLidarTargetDialog_(nullptr),
  pRefObservationDialog_(nullptr)
{
}

//==================================================================================================
LidarReferenceCalibrationGui::~LidarReferenceCalibrationGui()
{
}

//==================================================================================================
void LidarReferenceCalibrationGui::initializeGuiContents()
{
    CalibrationGuiBase::initializeGuiContents();

    //--- initialize content of placement guidance dialog
    if (pPlacementGuidanceDialog_)
    {
        pPlacementGuidanceDialog_->setFixedReferenceFrame((calibrationMetaData_.base_frame_id.empty())
                                                            ? calibrationMetaData_.ref_frame_id
                                                            : calibrationMetaData_.base_frame_id);
        pPlacementGuidanceDialog_->addAxes();
        pPlacementGuidanceDialog_->addRawSensorCloud(calibrationMetaData_.src_topic_name);
        pPlacementGuidanceDialog_->addGuidedPlacementBox(guidanceNodeletName_ +
                                                         "/" + PLACEMENT_GUIDANCE_TOPIC_NAME);

        if (calibrationMetaData_.base_frame_id.empty() == false)
            pPlacementGuidanceDialog_->setView(Rviz3dViewDialog::TOP_DOWN);
    }

    //--- initialize content of source lidar target visualization
    if (pSrcLidarTargetDialog_)
    {
        pSrcLidarTargetDialog_->setWindowTitle(
          QString::fromStdString(calibrationMetaData_.src_sensor_name));

        pSrcLidarTargetDialog_->setFixedReferenceFrame((calibrationMetaData_.base_frame_id.empty())
                                                         ? calibrationMetaData_.src_frame_id
                                                         : calibrationMetaData_.base_frame_id);
        pSrcLidarTargetDialog_->addAxes();
        pSrcLidarTargetDialog_->addRawSensorCloud(calibrationMetaData_.src_topic_name);
        pSrcLidarTargetDialog_
          ->addRegionsOfInterestCloud(calibratorNodeletName_ +
                                      "/" + calibrationMetaData_.src_sensor_name +
                                      "/" + ROIS_CLOUD_TOPIC_NAME);
        pSrcLidarTargetDialog_
          ->addCalibTargetCloud(calibratorNodeletName_ +
                                "/" + calibrationMetaData_.src_sensor_name +
                                "/" + TARGET_PATTERN_CLOUD_TOPIC_NAME);
        pSrcLidarTargetDialog_
          ->addMarkerCornersCloud(calibratorNodeletName_ +
                                  "/" + calibrationMetaData_.src_sensor_name +
                                  "/" + MARKER_CORNERS_TOPIC_NAME);
    }

    //--- initialize content of reference lidar target visualization
    if (pRefObservationDialog_)
    {
        pRefObservationDialog_->setWindowTitle(
          QString::fromStdString(calibrationMetaData_.ref_sensor_name));
        pRefObservationDialog_->setSensorName(calibrationMetaData_.ref_sensor_name);
    }

    //--- hide progress dialog
    hideProgressDialog();
}
//==================================================================================================
void LidarReferenceCalibrationGui::loadVisualizer()
{
}

//==================================================================================================
bool LidarReferenceCalibrationGui::setupGuiElements()
{
    bool isSuccessful = CalibrationGuiBase::setupGuiElements();
    if (!isSuccessful)
        return false;

    pCalibControlWindow_->setWindowTitle(
      QString::fromStdString(CALIB_TYPE_2_STR.at(EXTRINSIC_LIDAR_REFERENCE_CALIBRATION)) + " Calibration");

    pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled(false);

    //--- setup placement guidance dialog at the top-right corner of the display
    pPlacementGuidanceDialog_ = std::make_shared<Rviz3dViewDialog>(pCalibControlWindow_.get());
    if (!pPlacementGuidanceDialog_)
        return false;
    pPlacementGuidanceDialog_->setWindowTitle("Target Placement Guidance");
    pPlacementGuidanceDialog_->move(screenGeometry_.width() / 2, 0);
    pPlacementGuidanceDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                            (screenGeometry_.height() / 2) - titleBarHeight_ - 1);
    pCalibControlWindow_->attachPlacementGuidanceDialog(pPlacementGuidanceDialog_.get());
    pPlacementGuidanceDialog_->show();

    //--- setup source lidar target dialog at the bottom-left corner of the display
    pSrcLidarTargetDialog_ = std::make_shared<Rviz3dViewDialog>(pCalibControlWindow_.get());
    if (!pSrcLidarTargetDialog_)
        return false;
    pSrcLidarTargetDialog_->setWindowTitle("Source LiDAR Target Detections");
    pSrcLidarTargetDialog_->move(0, (screenGeometry_.height() / 2) + (2 * titleBarHeight_));
    pSrcLidarTargetDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                         (screenGeometry_.height() / 2) - titleBarHeight_ - 1);

    pCalibControlWindow_->attachSourceDialog(pSrcLidarTargetDialog_.get());
    pSrcLidarTargetDialog_->show();

    //--- setup reference lidar target dialog at the bottom-right corner of the display
    pRefObservationDialog_ =
      std::make_shared<ObservationsViewDialog>(this, pCalibControlWindow_.get());
    if (!pRefObservationDialog_)
        return false;
    pRefObservationDialog_->setWindowTitle("Reference");
    pRefObservationDialog_->move(screenGeometry_.width() / 2,
                                 (screenGeometry_.height() / 2) + (2 * titleBarHeight_));
    pRefObservationDialog_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                         (screenGeometry_.height() / 2) - titleBarHeight_ - 1);
    pCalibControlWindow_->attachReferenceDialog(pRefObservationDialog_.get());
    pRefObservationDialog_->show();

    //--- show infinite progress dialog at screen center
    showProgressDialog("Initializing user interface ...");

    return true;
}

} // namespace multisensor_calibration
