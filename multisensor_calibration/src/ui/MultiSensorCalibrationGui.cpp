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

#include "../include/multisensor_calibration/ui/MultiSensorCalibrationGui.h"

// Qt
#include <QApplication>
#include <QMessageBox>

// multisensor_calibration
#include "../include/multisensor_calibration/common/common.h"
#include "../include/multisensor_calibration/ui/CameraLidarCalibrationGui.h"
#include "../include/multisensor_calibration/ui/CameraReferenceCalibrationGui.h"
#include "../include/multisensor_calibration/ui/LidarLidarCalibrationGui.h"
#include "../include/multisensor_calibration/ui/LidarReferenceCalibrationGui.h"

namespace multisensor_calibration
{

//==================================================================================================
MultiSensorCalibrationGui::MultiSensorCalibrationGui(const std::string& iAppTitle,
                                                     const std::string& iGuiSubNamespace) :
  GuiBase(iAppTitle, iGuiSubNamespace),
  pCalibConfigDialog_(nullptr),
  pCalibrationGui_(nullptr)
{
}

//==================================================================================================
MultiSensorCalibrationGui::~MultiSensorCalibrationGui()
{
}

//==================================================================================================
void MultiSensorCalibrationGui::init()
{
    GuiBase::init();

    pCalibConfigDialog_ = std::make_shared<CalibrationConfigDialog>();

    connect(pCalibConfigDialog_.get(), &QDialog::accepted,
            this, &MultiSensorCalibrationGui::handleConfigDialogAccepted);
    connect(this, &GuiBase::rosLoopTerminated,
            this, &MultiSensorCalibrationGui::handleRosLoopTerminated);

    pCalibConfigDialog_->show();
}

//==================================================================================================
void MultiSensorCalibrationGui::runExtrinsicCameraLidarCalibration()
{
    if (!pNodeletLoader_)
        pNodeletLoader_ = std::make_shared<nodelet::Loader>();

    // Name of the calibration nodelet instance
    std::string calibratorName =
      ros::this_node::getName() + "/" + multisensor_calibration::CALIBRATOR_SUB_NAMESPACE;

    // Name of the guidance nodelet instance
    std::string guidanceName =
      ros::this_node::getName() + "/" + multisensor_calibration::GUIDANCE_SUB_NAMESPACE;

    //--- set parameters for calibration nodelet
    for (auto opt : pCalibConfigDialog_->getBoolTypedCalibrationOptions())
        ros::param::set(calibratorName + "/" + opt.first, opt.second);
    for (auto opt : pCalibConfigDialog_->getDoubleTypedCalibrationOptions())
        ros::param::set(calibratorName + "/" + opt.first, opt.second);
    for (auto opt : pCalibConfigDialog_->getIntTypedCalibrationOptions())
        ros::param::set(calibratorName + "/" + opt.first, opt.second);
    for (auto opt : pCalibConfigDialog_->getStringTypedCalibrationOptions())
        ros::param::set(calibratorName + "/" + opt.first, opt.second);

    //--- Load nodelets
    pNodeletLoader_->load(calibratorName,
                          "multisensor_calibration/ExtrinsicCameraLidarCalibrationNodelet", {}, {});
    pNodeletLoader_->load(guidanceName,
                          "multisensor_calibration/GuidedCameraLidarTargetPlacementNodelet", {}, {});

    //--- load gui
    pCalibrationGui_.reset(
      new CameraLidarCalibrationGui(appTitle_, multisensor_calibration::GUI_SUB_NAMESPACE));

    pCalibrationGui_->setNodeletLoaderPtr(pNodeletLoader_);
    pCalibrationGui_->init();
}

//==================================================================================================
void MultiSensorCalibrationGui::runExtrinsicCameraReferenceCalibration()
{
    if (!pNodeletLoader_)
        pNodeletLoader_ = std::make_shared<nodelet::Loader>();

    // Name of the calibration nodelet instance
    std::string calibratorName =
      ros::this_node::getName() + "/" + multisensor_calibration::CALIBRATOR_SUB_NAMESPACE;

    // Name of the guidance nodelet instance
    std::string guidanceName =
      ros::this_node::getName() + "/" + multisensor_calibration::GUIDANCE_SUB_NAMESPACE;

    //--- set parameters for calibration nodelet
    for (auto opt : pCalibConfigDialog_->getBoolTypedCalibrationOptions())
        ros::param::set(calibratorName + "/" + opt.first, opt.second);
    for (auto opt : pCalibConfigDialog_->getDoubleTypedCalibrationOptions())
        ros::param::set(calibratorName + "/" + opt.first, opt.second);
    for (auto opt : pCalibConfigDialog_->getIntTypedCalibrationOptions())
        ros::param::set(calibratorName + "/" + opt.first, opt.second);
    for (auto opt : pCalibConfigDialog_->getStringTypedCalibrationOptions())
        ros::param::set(calibratorName + "/" + opt.first, opt.second);

    //--- Load nodelets
    pNodeletLoader_->load(calibratorName,
                          "multisensor_calibration/ExtrinsicCameraReferenceCalibrationNodelet", {}, {});
    pNodeletLoader_->load(guidanceName,
                          "multisensor_calibration/GuidedCameraLidarTargetPlacementNodelet", {}, {});

    //--- load gui
    pCalibrationGui_.reset(
      new CameraReferenceCalibrationGui(appTitle_, multisensor_calibration::GUI_SUB_NAMESPACE));

    pCalibrationGui_->setNodeletLoaderPtr(pNodeletLoader_);
    pCalibrationGui_->init();
}

//==================================================================================================
void MultiSensorCalibrationGui::runExtrinsicLidarLidarCalibration()
{
    if (!pNodeletLoader_)
        pNodeletLoader_ = std::make_shared<nodelet::Loader>();

    // Name of the calibration nodelet instance
    std::string calibratorName =
      ros::this_node::getName() + "/" + multisensor_calibration::CALIBRATOR_SUB_NAMESPACE;

    // Name of the guidance nodelet instance
    std::string guidanceName =
      ros::this_node::getName() + "/" + multisensor_calibration::GUIDANCE_SUB_NAMESPACE;

    //--- set parameters for calibration nodelet
    for (auto opt : pCalibConfigDialog_->getBoolTypedCalibrationOptions())
        ros::param::set(calibratorName + "/" + opt.first, opt.second);
    for (auto opt : pCalibConfigDialog_->getDoubleTypedCalibrationOptions())
        ros::param::set(calibratorName + "/" + opt.first, opt.second);
    for (auto opt : pCalibConfigDialog_->getIntTypedCalibrationOptions())
        ros::param::set(calibratorName + "/" + opt.first, opt.second);
    for (auto opt : pCalibConfigDialog_->getStringTypedCalibrationOptions())
        ros::param::set(calibratorName + "/" + opt.first, opt.second);

    //--- Load nodelets
    pNodeletLoader_->load(calibratorName,
                          "multisensor_calibration/ExtrinsicLidarLidarCalibrationNodelet", {}, {});
    pNodeletLoader_->load(guidanceName,
                          "multisensor_calibration/GuidedLidarLidarTargetPlacementNodelet", {}, {});

    //--- load gui
    pCalibrationGui_.reset(
      new LidarLidarCalibrationGui(appTitle_, multisensor_calibration::GUI_SUB_NAMESPACE));

    pCalibrationGui_->setNodeletLoaderPtr(pNodeletLoader_);
    pCalibrationGui_->init();
}

//==================================================================================================
void MultiSensorCalibrationGui::runExtrinsicLidarReferenceCalibration()
{
    if (!pNodeletLoader_)
        pNodeletLoader_ = std::make_shared<nodelet::Loader>();

    // Name of the calibration nodelet instance
    std::string calibratorName =
      ros::this_node::getName() + "/" + multisensor_calibration::CALIBRATOR_SUB_NAMESPACE;

    // Name of the guidance nodelet instance
    std::string guidanceName =
      ros::this_node::getName() + "/" + multisensor_calibration::GUIDANCE_SUB_NAMESPACE;

    //--- set parameters for calibration nodelet
    for (auto opt : pCalibConfigDialog_->getBoolTypedCalibrationOptions())
        ros::param::set(calibratorName + "/" + opt.first, opt.second);
    for (auto opt : pCalibConfigDialog_->getDoubleTypedCalibrationOptions())
        ros::param::set(calibratorName + "/" + opt.first, opt.second);
    for (auto opt : pCalibConfigDialog_->getIntTypedCalibrationOptions())
        ros::param::set(calibratorName + "/" + opt.first, opt.second);
    for (auto opt : pCalibConfigDialog_->getStringTypedCalibrationOptions())
        ros::param::set(calibratorName + "/" + opt.first, opt.second);

    //--- Load nodelets
    pNodeletLoader_->load(calibratorName,
                          "multisensor_calibration/ExtrinsicLidarReferenceCalibrationNodelet", {}, {});
    pNodeletLoader_->load(guidanceName,
                          "multisensor_calibration/GuidedLidarLidarTargetPlacementNodelet", {}, {});

    //--- load gui
    pCalibrationGui_.reset(
      new LidarReferenceCalibrationGui(appTitle_, multisensor_calibration::GUI_SUB_NAMESPACE));

    pCalibrationGui_->setNodeletLoaderPtr(pNodeletLoader_);
    pCalibrationGui_->init();
}

//==================================================================================================
void MultiSensorCalibrationGui::handleConfigDialogAccepted()
{
    switch (pCalibConfigDialog_->selectedCalibrationType())
    {
    default:
    case EXTRINSIC_CAMERA_LIDAR_CALIBRATION:
    {
        runExtrinsicCameraLidarCalibration();
    }
    break;

    case EXTRINSIC_CAMERA_REFERENCE_CALIBRATION:
    {
        runExtrinsicCameraReferenceCalibration();
    }
    break;

    case EXTRINSIC_LIDAR_LIDAR_CALIBRATION:
    {
        runExtrinsicLidarLidarCalibration();
    }
    break;

    case EXTRINSIC_LIDAR_REFERENCE_CALIBRATION:
    {
        runExtrinsicLidarReferenceCalibration();
    }
    break;
    }
}

//==================================================================================================
void MultiSensorCalibrationGui::handleRosLoopTerminated()
{
    pCalibConfigDialog_->reject();
}

} // namespace multisensor_calibration