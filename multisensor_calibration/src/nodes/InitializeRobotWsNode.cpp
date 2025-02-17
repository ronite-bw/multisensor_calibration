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
#include <filesystem>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// multisensor_calibration
#include "../../include/multisensor_calibration/config/Workspace.h"

#if !defined(TARGET_NAME)
#define TARGET_NAME ""
#endif

namespace fs = std::filesystem;

/**
 * @brief Entry point for the 'initialize_robot_workspace' node.
 *
 * This will initialize a robot workspace with given information.
 *
 * <b>Launch-Parameters:</b>
 * - ```robot_ws_path```: Path to where the robot workspace is to be initialized.
 *    - Type: String
 *    - Default: ""
 * - ```robot_name```: Name of the robot to which the workspace corresponds.
 *    - Type: String
 *    - Default: ""
 * - ```urdf_model_path```: (Optional) Path to URDF model associated with the robot.
 *    - Type: String
 *    - Default: ""
 */
int main(int argc, char** argv)
{
    //---Initialize ROS node
    ros::init(argc, argv, TARGET_NAME);
    ros::NodeHandle nh("~");

    //--- Read launch parameters
    std::string robotWsPathStr; // string containing read robot_ws_path
    nh.param<std::string>("robot_ws_path", robotWsPathStr, "");
    if (robotWsPathStr.empty())
    {
        ROS_ERROR("[%s] None or empty path string passed to 'robot_ws_path'."
                  "\nPlease provide valid path to robot workspace.",
                  TARGET_NAME);
        return 1;
    }

    std::string robotName; // name of robot
    nh.param<std::string>("robot_name", robotName, "");
    if (robotName.empty())
    {
        ROS_ERROR("[%s] None or empty string passed to 'robot_name'."
                  "\nPlease provide valid robot name.",
                  TARGET_NAME);
        return 1;
    }

    std::string urdfModelPathStr; // string containing read robot_ws_path
    nh.param<std::string>("urdf_model_path", urdfModelPathStr, "");

    //--- initialize workspace
    multisensor_calibration::RobotWorkspace robotWs(robotWsPathStr);
    if (robotWs.isValid())
    {
        ROS_WARN("[%s] Robot workspace already exists and will not be reinitialized."
                 "\nWorkspace path: %s",
                 TARGET_NAME, robotWsPathStr.c_str());
        return 1;
    }
    else if (fs::exists(fs::absolute(robotWsPathStr)))
    {
        ROS_WARN("[%s] Given path to workspace already exists but is not a robot workspace."
                 "No initialization is performed."
                 "\nWorkspace path: %s",
                 TARGET_NAME, robotWsPathStr.c_str());
        return 1;
    }

    robotWs.initialize();
    robotWs.load();

    //--- save settings
    QSettings* pSettings = robotWs.settingsPtr();
    pSettings->setValue("robot/name", QString::fromStdString(robotName));
    pSettings->setValue("robot/urdf_model_path", QString::fromStdString(urdfModelPathStr));
    pSettings->sync();

    ROS_INFO("[%s] Robot workspace successfully initialized."
             "\nWorkspace path: %s",
             TARGET_NAME, robotWsPathStr.c_str());
    return 0;
}