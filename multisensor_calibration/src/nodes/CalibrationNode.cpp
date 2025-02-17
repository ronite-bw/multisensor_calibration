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

// Std
#include <iostream>
#include <locale>
#include <memory>

// ROS
#include <nodelet/loader.h>
#include <ros/ros.h>

// QT
#include <QApplication>

// multisensor_calibration
#include "../../include/multisensor_calibration/common/common.h"
#include "../../include/multisensor_calibration/ui/CalibrationGuiBase.h"
#include "../../include/multisensor_calibration/ui/CameraLidarCalibrationGui.h"
#include "../../include/multisensor_calibration/ui/CameraReferenceCalibrationGui.h"
#include "../../include/multisensor_calibration/ui/GuiBase.h"
#include "../../include/multisensor_calibration/ui/LidarLidarCalibrationGui.h"
#include "../../include/multisensor_calibration/ui/LidarReferenceCalibrationGui.h"
#include "../../include/multisensor_calibration/ui/MultiSensorCalibrationGui.h"

#if !defined(TARGET_NAME)
#define TARGET_NAME ""
#endif

/**
 * @brief Main entry point for all calibration programs.
 *
 * This will load the Gui application, as well as the calibration and guidance nodelets.
 * Depending on the preprocessor define TARGET, different configurations of the GUI and
 * nodelets are initialized.
 */

int main(int argc, char** argv)
{
    std::setlocale(LC_ALL, "en_US.UTF-8");

    //--- initialize ROS
    ros::init(argc, argv, TARGET_NAME);

    //--- initialize nodelet loader
    std::shared_ptr<nodelet::Loader> pNodeletLoader = std::make_shared<nodelet::Loader>();
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;

    //--- initialize Qt
    QApplication app(argc, argv);

    //--- Load nodelets and setup Gui
    std::shared_ptr<multisensor_calibration::GuiBase> pGui = nullptr;
#if TARGET == MULTI_SENSOR_CALIBRATION_TARGET

    /*
     * For the ROS node 'multisensor_calibration', only the calibration configurator is loaded.
     */

    pGui.reset(new multisensor_calibration::MultiSensorCalibrationGui(ros::this_node::getName(),
                                                                      multisensor_calibration::GUI_SUB_NAMESPACE));
#elif TARGET == EXTRINSIC_CAMERA_LIDAR_CALIBRATION_TARGET

    /*
     * For 'extrinsic_camera_lidar_calibration',
     *   - the ExtrinsicCameraLidarCalibrationNodelet,
     *   - the GuidedCameraLidarTargetPlacementNodelet, as well as
     *   - the CameraLidarCalibrationGui
     * is loaded.
     */

    pNodeletLoader->load(ros::this_node::getName() + "/" + multisensor_calibration::CALIBRATOR_SUB_NAMESPACE,
                         "multisensor_calibration/ExtrinsicCameraLidarCalibrationNodelet", remap, nargv);
    pNodeletLoader->load(ros::this_node::getName() + "/" + multisensor_calibration::GUIDANCE_SUB_NAMESPACE,
                         "multisensor_calibration/GuidedCameraLidarTargetPlacementNodelet", remap, nargv);
    pGui.reset(new multisensor_calibration::CameraLidarCalibrationGui(ros::this_node::getName(),
                                                                      multisensor_calibration::GUI_SUB_NAMESPACE));
#elif TARGET == EXTRINSIC_LIDAR_LIDAR_CALIBRATION_TARGET

    /*
     * For 'extrinsic_camera_lidar_calibration',
     *   - the ExtrinsicLidarLidarCalibrationNodelet,
     *   - the GuidedLidarLidarTargetPlacementNodelet, as well as
     *   - the LidarLidarCalibrationGui
     * is loaded.
     */

    pNodeletLoader->load(ros::this_node::getName() + "/" + multisensor_calibration::CALIBRATOR_SUB_NAMESPACE,
                         "multisensor_calibration/ExtrinsicLidarLidarCalibrationNodelet", remap, nargv);
    pNodeletLoader->load(ros::this_node::getName() + "/" + multisensor_calibration::GUIDANCE_SUB_NAMESPACE,
                         "multisensor_calibration/GuidedLidarLidarTargetPlacementNodelet", remap, nargv);
    pGui.reset(new multisensor_calibration::LidarLidarCalibrationGui(ros::this_node::getName(),
                                                                     multisensor_calibration::GUI_SUB_NAMESPACE));
#elif TARGET == EXTRINSIC_CAMERA_REFERENCE_CALIBRATION_TARGET

    /*
     * For 'extrinsic_camera_reference_calibration',
     *   - the ExtrinsicCameraReferenceCalibrationNodelet,
     *   - the GuidedCameraLidarTargetPlacementNodelet, as well as
     *   - the CameraReferenceCalibrationGui
     * is loaded.
     */

    pNodeletLoader->load(ros::this_node::getName() + "/" + multisensor_calibration::CALIBRATOR_SUB_NAMESPACE,
                         "multisensor_calibration/ExtrinsicCameraReferenceCalibrationNodelet", remap, nargv);
    pNodeletLoader->load(ros::this_node::getName() + "/" + multisensor_calibration::GUIDANCE_SUB_NAMESPACE,
                         "multisensor_calibration/GuidedCameraLidarTargetPlacementNodelet", remap, nargv);
    pGui.reset(new multisensor_calibration::CameraReferenceCalibrationGui(ros::this_node::getName(),
                                                                          multisensor_calibration::GUI_SUB_NAMESPACE));
#elif TARGET == EXTRINSIC_LIDAR_REFERENCE_CALIBRATION_TARGET

    /*
     * For 'extrinsic_lidar_reference_calibration',
     *   - the ExtrinsicLidarReferenceCalibrationNodelet,
     *   - the GuidedLidarLidarTargetPlacementNodelet, as well as
     *   - the LidarLidarCalibrationGui
     * is loaded.
     */

    pNodeletLoader->load(ros::this_node::getName() + "/" + multisensor_calibration::CALIBRATOR_SUB_NAMESPACE,
                         "multisensor_calibration/ExtrinsicLidarReferenceCalibrationNodelet", remap, nargv);
    pNodeletLoader->load(ros::this_node::getName() + "/" + multisensor_calibration::GUIDANCE_SUB_NAMESPACE,
                         "multisensor_calibration/GuidedLidarLidarTargetPlacementNodelet", remap, nargv);
    pGui.reset(new multisensor_calibration::LidarReferenceCalibrationGui(ros::this_node::getName(),
                                                                         multisensor_calibration::GUI_SUB_NAMESPACE));
#else
    std::cerr << "No valid TARGET passed as compiler define!" << std::endl;
    return 1;
#endif

    //--- initialize gui nodelet
    pGui->setNodeletLoaderPtr(pNodeletLoader);
    pGui->init();

    //--- run application
    int appRetVal = app.exec();

    return appRetVal;
}