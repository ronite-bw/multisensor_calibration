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

#include "../include/multisensor_calibration/calibration/CalibrationBase.h"

// Std
#include <iostream>
#include <string>

// Qt
#include <QFile>

// ROS
#include <ros/ros.h>

// PCL
#include <pcl/conversions.h>

// multisensor_calibration
#include "../include/multisensor_calibration/common/utils.hpp"

namespace multisensor_calibration
{

//==================================================================================================
CalibrationBase::CalibrationBase(ECalibrationType type) :
  type_(type),
  nodeletName_(""),
  isInitialized_(false),
  pRobotWs_(nullptr),
  robotName_(""),
  isUrdfModelAvailable_(false),
  urdfModelPath_(""),
  urdfModel_(),
  saveObservationsToWs_(false),
  calibTargetFilePath_(""),
  captureCalibrationTarget_(false),
  calibrationItrCnt_(1)
{
}

//==================================================================================================
CalibrationBase::~CalibrationBase()
{
}

//==================================================================================================
bool CalibrationBase::createAndStartCalibrationWorkflow(ros::NodeHandle& ioNh)
{
    bool isSuccessful = initializeTimers(ioNh);

    if (isSuccessful)
        loadRobotWsTrigger_.start();

    return isSuccessful;
}

//==================================================================================================
bool CalibrationBase::initializeDataProcessors()
{

    return true;
}

//==================================================================================================
bool CalibrationBase::initializePublishers(ros::NodeHandle& ioNh)
{
    calibResultPub_ = ioNh.advertise<CalibrationResult_Message_T>(
      CALIB_RESULT_TOPIC_NAME, 10);

    return true;
}

//==================================================================================================
bool CalibrationBase::initializeServices(ros::NodeHandle& ioNh)
{
    //--- capture target service
    captureSrv_ = ioNh.advertiseService(
      CAPTURE_TARGET_SRV_NAME,
      &CalibrationBase::onRequestTargetCapture, this);

    //--- finalize calibration
    finalizeSrv_ = ioNh.advertiseService(
      FINALIZE_CALIBRATION_SRV_NAME,
      &CalibrationBase::onRequestCalibrationFinalization, this);

    //--- reset service
    resetSrv_ = ioNh.advertiseService(
      RESET_SRV_NAME,
      &CalibrationBase::onReset, this);

    return true;
}

//==================================================================================================
bool CalibrationBase::initializeWorkspaceObjects()
{
    //--- initialize robot workspace
    pRobotWs_ = std::make_shared<RobotWorkspace>(robotWsPath_, nodeletName_);

    return (pRobotWs_ != nullptr);
}

//==================================================================================================
bool CalibrationBase::isFrameIdInUrdfModel(const std::string& iFrameId) const
{
    bool isSensorInModel = (urdfModel_.getLink(iFrameId) != nullptr);

    return isSensorInModel;
}

//==================================================================================================
bool CalibrationBase::saveCalibrationSettingsToWorkspace()
{
    QSettings* pCalibSettings = pCalibrationWs_->settingsPtr();
    if (!pCalibSettings)
        return false;

    //--- target_config fil
    std::string fileName = calibTargetFilePath_.filename().string();
    pCalibSettings->setValue("calibration/target_config_file",
                             QString::fromStdString(fileName));

    //--- initial guess
    pCalibSettings->setValue("calibration/save_observations",
                             QVariant::fromValue(saveObservationsToWs_));

    //--- copy calibration target file
    QFile calibTargetFile(QString::fromStdString(calibTargetFilePath_.c_str()));
    QString newFilePath = QString::fromStdString(pCalibrationWs_->getPath()) +
                          fs::path::preferred_separator +
                          calibTargetFile.fileName()
                            .split(fs::path::preferred_separator)
                            .back();
    if (QFile(newFilePath).exists())
        QFile::remove(newFilePath);

    bool isSuccesful = true;
    isSuccesful &= calibTargetFile.copy(newFilePath);
    isSuccesful &= calibTargetFile.setPermissions(QFileDevice::ReadOwner | QFileDevice::WriteOwner |
                                                  QFileDevice::ReadGroup | QFileDevice::WriteGroup |
                                                  QFileDevice::ReadOther);

    return isSuccesful;
}

//==================================================================================================
bool CalibrationBase::readLaunchParameters(const ros::NodeHandle& iNh)
{
    ros::V_string params;
    iNh.getParamNames(params);

    //--- robot workspace path
    std::string robotWsPathStr; // string containing read robot_ws_path
    iNh.param<std::string>("robot_ws_path", robotWsPathStr, "");
    if (robotWsPathStr.empty())
    {
        ROS_ERROR("[%s] None or empty path string passed to 'robot_ws_path'."
                  "\nPlease provide valid path to robot workspace.",
                  nodeletName_.c_str());
        return false;
    }
    robotWsPath_ = fs::absolute(robotWsPathStr);

    // path to target configuration file
    std::string targetFileStr;
    iNh.param<std::string>("target_config_file", targetFileStr, std::string(""));
    if (targetFileStr.empty() || !fs::exists(targetFileStr))
    {
        ROS_ERROR("[%s] Target configuration file path is empty or does not consist: %s",
                  nodeletName_.c_str(), targetFileStr.c_str());
        return false;
    }
    calibTargetFilePath_ = fs::absolute(targetFileStr);

    //--- save observation to workspace
    saveObservationsToWs_ = iNh.param<bool>("save_observations", false);

    return true;
}

//==================================================================================================
template <typename T>
T CalibrationBase::readNumericLaunchParameter(const ros::NodeHandle& iNh,
                                              const std::string& iParamName,
                                              const T& iDefaultVal,
                                              const T& iMinVal,
                                              const T& iMaxVal) const
{
    T num = 0;
    iNh.param<T>(iParamName, num, iDefaultVal);
    if (num < iMinVal)
    {
        ROS_WARN("[%s]"
                 "\n\t> (%s < %i) Setting %s to default: %i",
                 nodeletName_.c_str(), iParamName.c_str(),
                 iMinVal, iParamName.c_str(), iDefaultVal);
        num = iDefaultVal;
    }
    else if (num > iMaxVal)
    {
        ROS_WARN("[%s]"
                 "\n\t> (%s > %i) Setting %s to default: %i",
                 nodeletName_.c_str(), iParamName.c_str(),
                 iMaxVal, iParamName.c_str(), iDefaultVal);
        num = iDefaultVal;
    }

    return num;
}
template int CalibrationBase::readNumericLaunchParameter<int>(const ros::NodeHandle&,
                                                              const std::string&,
                                                              const int&,
                                                              const int&,
                                                              const int&) const;

//==================================================================================================
std::string CalibrationBase::readStringLaunchParameter(const ros::NodeHandle& iNh,
                                                       const std::string& iParamName,
                                                       const std::string& iDefaultVal) const
{

    std::string str = "";
    iNh.param<std::string>(iParamName, str, iDefaultVal);

    //--- if passed string is empty and default value is not empty, it is assumed that the parameter
    //--- is not allowed to be empty.
    if (str.empty() && !iDefaultVal.empty())
    {
        ROS_WARN("[%s]"
                 "\n\t> Empty string passed to '%s'. "
                 "\n\t> Setting '%s' to default: %s",
                 nodeletName_.c_str(), iParamName.c_str(),
                 iParamName.c_str(), iDefaultVal.c_str());
        str = iDefaultVal;
    }

    return str;
}

//==================================================================================================
void CalibrationBase::reset()
{
    calibrationItrCnt_ = 1;
}

//==================================================================================================
bool CalibrationBase::initializeTimers(ros::NodeHandle& ioNh)
{
    //--- initialize trigger to call routine to load the robot workspace
    loadRobotWsTrigger_ = ioNh.createTimer(
      ros::Duration(0.5), &CalibrationBase::onLoadRobotWorkspaceTriggered,
      this, true, false);

    //--- initialize trigger to call routine to load URDF model of robot
    loadUrdfModelTrigger_ = ioNh.createTimer(
      ros::Duration(0.5), &CalibrationBase::onLoadRobotUrdfModelTriggered,
      this, true, false);

    //--- initialize trigger to call routine to load the calibration workspace
    loadCalibrationWsTrigger_ = ioNh.createTimer(
      ros::Duration(0.5), &CalibrationBase::onLoadCalibrationWorkspaceTriggered,
      this, true, false);

    //--- initialize trigger to call routine to initialize and start data processing
    startSensorCalibrationTrigger_ = ioNh.createTimer(
      ros::Duration(0.5), &CalibrationBase::onStartSensorCalibrationTriggered,
      this, true, false);

    //--- initialize trigger to finalize calibration
    finalizeCalibrationTrigger_ = ioNh.createTimer(
      ros::Duration(0.5), &CalibrationBase::onFinalizeCalibrationTriggered,
      this, true, false);

    return true;
}

//==================================================================================================
void CalibrationBase::onFinalizeCalibrationTriggered(const ros::TimerEvent&)
{
    //--- reset trigger
    finalizeCalibrationTrigger_.stop();

    //--- finalize calibration
    finalizeCalibration();

    //--- save calibration
    saveCalibration();
}

//==================================================================================================
void CalibrationBase::onLoadCalibrationWorkspaceTriggered(const ros::TimerEvent&)
{
    //--- reset trigger
    loadCalibrationWsTrigger_.stop();

    //--- assert that the pointer to the calibration workspace is not null
    assert(pCalibrationWs_ != nullptr);

    //--- get settings object from workspace
    bool isSuccessful = pCalibrationWs_->load(true);

    //--- if not successful check to start again
    if (!isSuccessful)
    {
        ROS_ERROR("[%s] Loading of calibration workspace was not successful. Path: %s.",
                  nodeletName_.c_str(), pCalibrationWs_->getPath().c_str());
        isInitialized_ = false;
        return;
    }

    //--- save calibration settings to workspace
    isSuccessful = saveCalibrationSettingsToWorkspace();

    isInitialized_ &= true;

    ROS_INFO("[%s] Successfully loaded calibration workspace. Path: %s.",
             nodeletName_.c_str(), pCalibrationWs_->getPath().c_str());

    //--- start trigger of next phase, i.e. starting sensor calibration
    startSensorCalibrationTrigger_.start();
}

//==================================================================================================
void CalibrationBase::onLoadRobotWorkspaceTriggered(const ros::TimerEvent&)
{
    //--- reset trigger
    loadRobotWsTrigger_.stop();

    //--- assert that the pointer to the robot workspace is not null
    assert(pRobotWs_ != nullptr);

    //--- get settings object from workspace
    bool isSuccessful = pRobotWs_->load();

    //--- if not successful check to start again
    if (!isSuccessful)
    {
        ROS_ERROR("[%s] Loading of robot workspace was not successful.",
                  nodeletName_.c_str());
        isInitialized_ = false;

        return;
    }

    //--- read and check contents of settings file
    isSuccessful = readRobotSettings();
    if (!isSuccessful)
    {
        isInitialized_ = false;
        return;
    }

    isInitialized_ &= true;

    ROS_INFO("[%s] Successfully loaded robot workspace. Path: %s.",
             nodeletName_.c_str(), pRobotWs_->getPath().c_str());

    //--- start trigger of next phase, i.e. loading of urdf model or calibration workspace
    if (isUrdfModelAvailable_)
        loadUrdfModelTrigger_.start();
    else
        loadCalibrationWsTrigger_.start();
}

//==================================================================================================
void CalibrationBase::onLoadRobotUrdfModelTriggered(const ros::TimerEvent&)
{
    //--- reset trigger
    loadUrdfModelTrigger_.stop();

    //--- Read URDF model into XML Doc (needed for later manipulation)
    urdfModelDoc_.LoadFile(urdfModelPath_.string().c_str());

    //--- initialized model from XML Doc
    bool isSuccessful = urdfModel_.initXml(&urdfModelDoc_);

    //--- if not successful check to start again
    if (!isSuccessful)
    {
        ROS_ERROR("[%s] Error in reading URDF model from file."
                  "\nModel file: %s",
                  nodeletName_.c_str(), urdfModelPath_.string().c_str());

        isInitialized_ = false;
        return;
    }

    isInitialized_ &= true;

    ROS_INFO("[%s] Successfully parsed URDF model from file. Path: %s.",
             nodeletName_.c_str(), urdfModelPath_.c_str());

    //--- start trigger of next phase, i.e. loading of calibration workspace
    loadCalibrationWsTrigger_.start();
}

//==================================================================================================
bool CalibrationBase::onRequestCalibrationFinalization(
  multisensor_calibration::FinalizeCalibration::Request& iReq,
  multisensor_calibration::FinalizeCalibration::Response& oRes)
{
    UNUSED_VAR(iReq);

    if (!isInitialized_)
        return false;

    //--- if calibration counter is greater than 1, i.e. if at least one calibration iteration
    //--- has been performed, trigger finalization. otherwise, do not trigger
    if (calibrationItrCnt_ > 1)
    {
        ROS_INFO("[%s] Finalizing calibration.",
                 nodeletName_.c_str());

        finalizeCalibrationTrigger_.start();

        oRes.isAccepted = true;
        oRes.msg        = "Finalizing calibration.";
    }
    else
    {
        oRes.isAccepted = false;
        oRes.msg        = "No calibration available.";
    }
    return true;
}

//==================================================================================================
bool CalibrationBase::onRequestTargetCapture(
  multisensor_calibration::CaptureCalibTarget::Request& iReq,
  multisensor_calibration::CaptureCalibTarget::Response& oRes)
{
    UNUSED_VAR(iReq);

    if (!isInitialized_)
        return false;

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(dataProcessingMutex_);

    //--- store request
    captureCalibrationTarget_ = true;

    //--- write response
    ROS_INFO("[%s] Start looking for calibration target!",
             nodeletName_.c_str());

    oRes.msg = "Start looking for calibration target!";

    oRes.isAccepted = true;

    return true;
}

//==================================================================================================
bool CalibrationBase::onReset(multisensor_calibration::ResetCalibration::Request& iReq,
                              multisensor_calibration::ResetCalibration::Response& oRes)
{
    UNUSED_VAR(iReq);

    reset();

    oRes.isAccepted = true;
    oRes.msg        = "Calibration is reset.";

    return true;
}

//==================================================================================================
void CalibrationBase::onStartSensorCalibrationTriggered(const ros::TimerEvent&)
{
    //--- reset trigger
    startSensorCalibrationTrigger_.stop();

    //--- initialize sensor data processing
    bool isSuccessful = initializeDataProcessors();

    //--- if not successful, print warning and return
    if (!isSuccessful)
    {
        ROS_ERROR("[%s]"
                  "\n\t> Error in the initialization of the sensor data processing!",
                  nodeletName_.c_str());
        return;
    }

    //--- initialize subscribers
    isSuccessful = initializeSubscribers(nh_);

    //--- if not successful, print warning and return
    if (!isSuccessful)
    {
        ROS_ERROR("[%s]"
                  "\n\t> Error in the initialization of subscribers!",
                  nodeletName_.c_str());
        return;
    }

    //--- initialize publishers
    isSuccessful = initializePublishers(pnh_);

    //--- if not successful, print warning and return
    if (!isSuccessful)
    {
        ROS_ERROR("[%s]"
                  "\n\t> Error in the initialization of publishers!",
                  nodeletName_.c_str());
        return;
    }

    isInitialized_ &= true;

    ROS_INFO("[%s] Successfully initialized processing of sensor data.",
             nodeletName_.c_str());
}

//==================================================================================================
bool CalibrationBase::readRobotSettings()
{
    bool retVal = true;

    // Pointer to robot settings
    QSettings* pRobotSettings = pRobotWs_->settingsPtr();
    if (!pRobotSettings)
        return false;

    // Lambda function to read string parameter with given name (iParamName)..
    // If parameter string is empty, 'isValid' is set to false
    auto readStringParameter = [&](const std::string& iParamName, const bool& isEmptyAllowed,
                                   bool& isValid) -> std::string
    {
        std::string str =
          pRobotSettings->value(QString::fromStdString(iParamName), "").toString().toStdString();
        if (str.empty() && !isEmptyAllowed)
        {
            ROS_ERROR("[%s]Value provided for '%s' in settings file is empty."
                      "\nSettings file: %s"
                      "\nPlease provide valid string.",
                      nodeletName_.c_str(),
                      iParamName.c_str(),
                      pRobotSettings->fileName().toStdString().c_str());
            isValid = false;
        }
        else
        {
            isValid = true;
        }

        return str;
    };

    //--- robot name
    robotName_ = readStringParameter("robot/name", false, retVal);

    //--- urdf model path
    std::string pathStr = readStringParameter("robot/urdf_model_path", true, retVal);
    fs::path tmpPath    = fs::path(pathStr);
    if (tmpPath.is_relative())
    {
        urdfModelPath_ = pRobotWs_->getPath();
        urdfModelPath_ /= tmpPath;
    }
    else
    {
        urdfModelPath_ = tmpPath;
    }
    isUrdfModelAvailable_ = (!pathStr.empty() && fs::exists(urdfModelPath_));
    if (!isUrdfModelAvailable_)
    {
        isUrdfModelAvailable_ = false;
        ROS_INFO("[%s] URDF Model is not available",
                 nodeletName_.c_str());
        if (!pathStr.empty())
        {
            ROS_WARN("\t> URDF file: %s"
                     "\n\t> Please provide valid path (absolute or relative) to URDF model file.",
                     tmpPath.c_str());
        }
    }

    return retVal;
}

} // namespace multisensor_calibration