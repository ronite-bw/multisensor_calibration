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

#include "../include/multisensor_calibration/ui/CameraReferenceCalibrationGui.h"

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
CameraReferenceCalibrationGui::CameraReferenceCalibrationGui(const std::string& iAppTitle,
                                                             const std::string& iGuiSubNamespace) :
  CalibrationGuiBase(iAppTitle, iGuiSubNamespace),
  pPlacementGuidanceDialog_(nullptr),
  pCameraTargetDialog_(nullptr),
  pRefObservationDialog_(nullptr)
{
}

//==================================================================================================
CameraReferenceCalibrationGui::~CameraReferenceCalibrationGui()
{
}

//==================================================================================================
void CameraReferenceCalibrationGui::initializeGuiContents()
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
void CameraReferenceCalibrationGui::loadVisualizer()
{
}

//==================================================================================================
bool CameraReferenceCalibrationGui::setupGuiElements()
{
    bool isSuccessful = CalibrationGuiBase::setupGuiElements();
    if (!isSuccessful)
        return false;

    pCalibControlWindow_->setWindowTitle(
      QString::fromStdString(CALIB_TYPE_2_STR.at(EXTRINSIC_CAMERA_REFERENCE_CALIBRATION)) + " Calibration");

    pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled(false);

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
