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

#ifndef MULTISENSORCALIBRATION_CALIBRATIONBASE_H
#define MULTISENSORCALIBRATION_CALIBRATIONBASE_H

// Std
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Qt
#include <QSettings>

// ROS
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/timer.h>
#include <tf/transform_listener.h>
#include <urdf/model.h>

// Tiny XML
#include <tinyxml2.h>

// multisensor_calibration
#include "../common/common.h"
#include "../common/lib3D/core/extrinsics.hpp"
#include "../config/Workspace.h"
#include "../data_processing/CameraDataProcessor.h"
#include "../data_processing/LidarDataProcessor.h"
#include <multisensor_calibration/CaptureCalibTarget.h>
#include <multisensor_calibration/FinalizeCalibration.h>
#include <multisensor_calibration/ResetCalibration.h>

namespace fs = std::filesystem;

namespace multisensor_calibration
{

/**
 * @ingroup calibration
 * @brief Base class of all calibration nodelets. This holds a common interface to all calibration
 * nodelets.
 */
class CalibrationBase
{
    //--- METHOD DECLARATION ---//
  public:
    /**
     * @brief Default constructor is deleted
     */
    CalibrationBase() = delete;

    /**
     * @brief Initialization constructor
     *
     * @param[in] type Type of calibration.
     */
    CalibrationBase(ECalibrationType type);

    /**
     * @brief Destructor
     */
    virtual ~CalibrationBase();

  protected:
    /**
     * @brief Create and start the calibration workflow.
     *
     * This will initialize the timers that trigger the different stages within the workflow.
     * After initialization the first stage (i.e. loading of the robot workspace) is triggered.
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    bool createAndStartCalibrationWorkflow(ros::NodeHandle& ioNh);

    /**
     * @brief Finalize calibration.
     *
     * This is purely virtual and intended as an interface. It needs to be implemented by all
     * classes that are to be instantiated. In this, the final calibration should be calculated.
     */
    virtual void finalizeCalibration() = 0;

    /**
     * @brief Initialize data processing, i.e. the objects that will process the sensor data and
     * detect the calibration target within the data.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    virtual bool initializeDataProcessors();

    /**
     * @brief Initialize publishers.
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    virtual bool initializePublishers(ros::NodeHandle& ioNh);

    /**
     * @brief Initialize services.
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    virtual bool initializeServices(ros::NodeHandle& ioNh);

    /**
     * @brief Initialize subscribers. Purely virtual.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    virtual bool initializeSubscribers(ros::NodeHandle& ioNh) = 0;

    /**
     * @brief Initialize workspace objects. In this class, i.e. the calibration base class, only the
     * object of the robot workspace is initialized. The initialization requires the launch
     * parameters, thus it is to be executed after the launch parameters are read.
     *
     * @return True if successful. False, otherwise (e.g. if instantiation has failed)
     */
    virtual bool initializeWorkspaceObjects();

    /**
     * @brief Check if given frame ID is in URDF model.
     *
     * @return True, if frame ID is found in model. False, otherwise.
     */
    bool isFrameIdInUrdfModel(const std::string& iFrameId) const;

    /**
     * @brief Save calibration settings to setting.ini inside calibration workspace.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    virtual bool saveCalibrationSettingsToWorkspace();

    /**
     * @brief Read launch parameters.
     *
     * The implementation within this class hold launch parameters that are common to all
     * calibration nodelets, e.g. robot_ws_path, target_config_file.
     *
     * @param[in] iNh Object of node handle
     * @return True if successful. False, otherwise (e.g. if sanity check fails)
     */
    virtual bool readLaunchParameters(const ros::NodeHandle& iNh);

    /**
     * @brief Read numeric launch parameter with given name ('iParamName') and default
     * Value ('iDefaultVal').
     *
     * If parameter value is below 'iMinVal' or above 'iMaxVal', the default value is set
     *
     * @return Parameter value.
     */
    template <typename T>
    T readNumericLaunchParameter(const ros::NodeHandle& iNh,
                                 const std::string& iParamName,
                                 const T& iDefaultVal,
                                 const T& iMinVal,
                                 const T& iMaxVal) const;

    /**
     * @brief Read string launch parameter with given name ('iParamName') and default
     * Value ('iDefaultVal').
     *
     * If parameter string is empty and default value is not empty, it is assumed that the parameter
     * is not allowed to be empty. In this case the default value will be set.
     *
     * @return Parameter value.
     */
    std::string readStringLaunchParameter(const ros::NodeHandle& iNh,
                                          const std::string& iParamName,
                                          const std::string& iDefaultVal = "") const;

    /**
     * @brief Reset calibration
     */
    virtual void reset();

    /**
     * @brief Shutdown subscribers and disconnect callbacks. Purely virtual.
     *
     * @return True, if successful. False, otherwise.
     */
    virtual bool shutdownSubscribers() = 0;

    /**
     * @brief Save calibration. Purely virtual.
     */
    virtual void saveCalibration() = 0;

  private:
    /**
     * @brief Initialize timers that will trigger the different steps within the calibration
     * workflow.
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    bool initializeTimers(ros::NodeHandle& ioNh);

    /**
     * @brief Response method when finalization of calibration is triggered.
     * Intended to be called by ros::Timer.
     */
    void onFinalizeCalibrationTriggered(const ros::TimerEvent&);

    /**
     * @brief Response method when loading of calibration workspace is triggered.
     * Intended to be called by ros::Timer.
     */
    void onLoadCalibrationWorkspaceTriggered(const ros::TimerEvent&);

    /**
     * @brief Response method when loading of robot workspace is triggered.
     * Intended to be called by ros::Timer.
     */
    void onLoadRobotWorkspaceTriggered(const ros::TimerEvent&);

    /**
     * @brief Response method when loading of robot urdf model is triggered.
     * Intended to be called by ros::Timer.
     */
    void onLoadRobotUrdfModelTriggered(const ros::TimerEvent&);

    /**
     * @brief Service call to request finalization of calibration
     *
     * @param[in] iReq Request, with flag to finalize calibration
     * @param[out] oRes Response, empty.
     */
    bool onRequestCalibrationFinalization(multisensor_calibration::FinalizeCalibration::Request& iReq,
                                          multisensor_calibration::FinalizeCalibration::Response& oRes);

    /**
     * @brief Service call to request capturing of calibration target
     *
     * @param[in] iReq Request, with flag to capture calibration target
     * @param[out] oRes Response, empty.
     */
    bool onRequestTargetCapture(multisensor_calibration::CaptureCalibTarget::Request& iReq,
                                multisensor_calibration::CaptureCalibTarget::Response& oRes);

    /**
     * @brief Service call to request reset of calibration
     *
     * @param[in] iReq Request, empty
     * @param[out] oRes Response.
     */
    bool onReset(multisensor_calibration::ResetCalibration::Request& iReq,
                 multisensor_calibration::ResetCalibration::Response& oRes);

    /**
     * @brief Response method when starting of sensor calibration is triggered.
     * Intended to be called by ros::Timer.
     */
    void onStartSensorCalibrationTriggered(const ros::TimerEvent&);

    /**
     * @brief Method to read robot specific settings from settings file within robot workspace.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    bool readRobotSettings();

    //--- MEMBER DECLARATION ---//

  protected:
    /// Type of calibration.
    ECalibrationType type_;

    /// Name of nodelet.
    std::string nodeletName_;

    /// Flag indicating if nodelet is initialized.
    bool isInitialized_;

    /// Mutex guarding the data processing.
    std::mutex dataProcessingMutex_;

    /// Global node handler.
    ros::NodeHandle nh_;

    /// Private node handler.
    ros::NodeHandle pnh_;

    /// Transform listener to get transform between the two sensor frames.
    tf::TransformListener tfListener_;

    /// Object to publish calibration result.
    ros::Publisher calibResultPub_;

    /// Server to provide service to request capturing of calibration target.
    ros::ServiceServer captureSrv_;

    /// Server to provide service to request finalization of calibration.
    ros::ServiceServer finalizeSrv_;

    /// Server to provide service to reset calibration.
    ros::ServiceServer resetSrv_;

    /// Absolute path to robot workspace.
    fs::path robotWsPath_;

    /// Pointer to robot workspace.
    std::shared_ptr<AbstractWorkspace> pRobotWs_;

    /// Name of robot.
    std::string robotName_;

    /// Flag to indicate if URDF model is available
    bool isUrdfModelAvailable_;

    /// absolute path to URDF model file of robot.
    fs::path urdfModelPath_;

    /// XML document of URDF model.
    tinyxml2::XMLDocument urdfModelDoc_;

    /// URDF model of robot.
    urdf::Model urdfModel_;

    /// Flag controlling whether the observations are to be saved into the calibration workspace
    /// after calibration.
    bool saveObservationsToWs_;

    /// Pointer to base calibration workspace. The instantiation of the pointer is done in the
    /// individual calibration nodelet.
    std::shared_ptr<AbstractWorkspace> pCalibrationWs_;

    /// Path to target configuration file
    fs::path calibTargetFilePath_;

    /// Object of timer to call routine to load robot workspace.
    /// The timer will be initialized as trigger with a period of 0.5seconds and a one-shot policy.
    ros::Timer loadRobotWsTrigger_;

    /// Object of timer to call routine to load URDF model of robot.
    /// The timer will be initialized as trigger with a period of 0.5seconds and a one-shot policy.
    ros::Timer loadUrdfModelTrigger_;

    /// Object of timer to call routine to load calibration workspace.
    /// The timer will be initialized as trigger with a period of 0.5seconds and a one-shot policy.
    ros::Timer loadCalibrationWsTrigger_;

    /// Object of timer to call routine to start sensor calibration.
    /// The timer will be initialized as trigger with a period of 0.5seconds and a one-shot policy.
    ros::Timer startSensorCalibrationTrigger_;

    /// Object of timer to call routine to finalize calibration.
    /// The timer will be initialized as trigger with a period of 0.5seconds and a one-shot policy.
    ros::Timer finalizeCalibrationTrigger_;

    /// Flag to capture calibration target in next sensor data package.
    bool captureCalibrationTarget_;

    /// Iteration number of the calibration routine.
    uint calibrationItrCnt_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_CALIBRATIONBASE_H