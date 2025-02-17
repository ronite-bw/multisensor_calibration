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

#include "../include/multisensor_calibration/ui/CalibrationGuiBase.h"

// Std
#include <future>
#include <regex>

// Qt
#include <QApplication>
#include <QDesktopServices>
#include <QDesktopWidget>
#include <QDirIterator>
#include <QFileDialog>
#include <QLocale>
#include <QMessageBox>
#include <QScreen>
#include <QStyle>
#include <QUrl>

// ROS
#include <ros/service_client.h>

// multisensor_calibration
#include "../include/multisensor_calibration/common/common.h"
#include "../include/multisensor_calibration/data_processing/DataProcessor3d.h"
#include "ui_CalibrationControlWindow.h"
#include <multisensor_calibration/CaptureCalibTarget.h>
#include <multisensor_calibration/FinalizeCalibration.h>
#include <multisensor_calibration/ImportMarkerObservations.h>
#include <multisensor_calibration/RemoveLastObservation.h>
#include <multisensor_calibration/ResetCalibration.h>

namespace multisensor_calibration
{

//==================================================================================================
CalibrationGuiBase::CalibrationGuiBase(const std::string& iAppTitle,
                                       const std::string& iGuiSubNamespace) :
  GuiBase(iAppTitle, iGuiSubNamespace),
  calibratorNodeletName_(""),
  guidanceNodeletName_(""),
  visualizerNodeletName_(""),
  pCalibControlWindow_(nullptr),
  pProgressDialog_(nullptr),
  pRqtReconfigureProcess_(nullptr)
{
    //--- initialize calibration meta data timer
    calibMetaDataTimer_.setInterval(1000);
    calibMetaDataTimer_.setSingleShot(false);
    connect(&calibMetaDataTimer_, &QTimer::timeout, this, &CalibrationGuiBase::getCalibrationMetaData);
}

//==================================================================================================
CalibrationGuiBase::~CalibrationGuiBase()
{
}

//==================================================================================================
std::string CalibrationGuiBase::getCalibratorNodeletName() const
{
    return calibratorNodeletName_;
}

//==================================================================================================
void CalibrationGuiBase::init()
{
    GuiBase::init();

    //--- set component names
    calibratorNodeletName_ = appTitle_ + "/" + CALIBRATOR_SUB_NAMESPACE;
    guidanceNodeletName_   = appTitle_ + "/" + GUIDANCE_SUB_NAMESPACE;
    visualizerNodeletName_ = appTitle_ + "/" + VISUALIZER_SUB_NAMESPACE;

    //--- initialize subscribers
    isInitialized_ &= initializeSubscribers(nh_);

    //--- setup GUI
    isInitialized_ &= setupGuiElements();

    //--- start ros event loop
    if (isInitialized_)
    {
        calibMetaDataTimer_.start();
    }
}

//==================================================================================================
void CalibrationGuiBase::setNodeletLoaderPtr(std::shared_ptr<nodelet::Loader>& ipLoader)
{
    GuiBase::setNodeletLoaderPtr(ipLoader);

    if (pCalibControlWindow_)
        pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled((pNodeletLoader_ != nullptr));
}

//==================================================================================================
void CalibrationGuiBase::hideProgressDialog()
{
    //--- update mouse cursor
    QApplication::restoreOverrideCursor();

    //--- update progress dialog
    if (pProgressDialog_)
        pProgressDialog_->reset();
}

//==================================================================================================
void CalibrationGuiBase::initializeGuiContents()
{

    //--- add submenue to import observations
    QString importObsToolTip = "Import observations of the calibration target from directory to the "
                               "corresponding sensor. This will remove all previously added/captured "
                               "observations.";
    QMenu* pImportObsSubMenu = new QMenu(pCalibControlWindow_.get());
    pImportObsSubMenu->setToolTip(importObsToolTip);

    // function to add action to import observations for given sensor name
    auto addImportAction = [&](const std::string& sensorName)
    {
        QAction* pImportObsAction = new QAction(
          QString("to '%1'").arg(QString::fromStdString(sensorName)),
          pCalibControlWindow_.get());
        pImportObsAction->setToolTip(importObsToolTip);
        pImportObsAction->setData(
          QVariant(QString::fromStdString(sensorName)));
        connect(pImportObsAction, &QAction::triggered,
                this, &CalibrationGuiBase::onActionImportObservationsTriggered);

        pImportObsSubMenu->addAction(pImportObsAction);
    };

    //--- only add importation for source sensor if it is not a camera sensor, as this is currently
    //--- not implemented.
    if (calibrationMetaData_.calib_type != static_cast<int>(EXTRINSIC_CAMERA_LIDAR_CALIBRATION) &&
        calibrationMetaData_.calib_type != static_cast<int>(EXTRINSIC_CAMERA_REFERENCE_CALIBRATION))
    {
        addImportAction(calibrationMetaData_.src_sensor_name);
    }

    addImportAction(calibrationMetaData_.ref_sensor_name);

    pCalibControlWindow_->actionImportObservationPtr()->setMenu(pImportObsSubMenu);
}

//==================================================================================================
bool CalibrationGuiBase::initializeSubscribers(ros::NodeHandle& ioNh)
{
    //--- subscribe to log messages
    logSubsc_ = ioNh.subscribe<rosgraph_msgs::Log>(
      "/rosout", 10,
      boost::bind(&CalibrationGuiBase::onLogMessageReceived, this, _1));

    //--- subscriber to calib result
    calibResultSubsc_ = ioNh.subscribe<CalibrationResult_Message_T>(
      calibratorNodeletName_ + "/" + CALIB_RESULT_TOPIC_NAME, 1,
      boost::bind(&CalibrationGuiBase::onCalibrationResultReceived, this, _1));

    return true;
}

//==================================================================================================
void CalibrationGuiBase::onCalibrationResultReceived(
  const CalibrationResult_Message_T::ConstPtr& ipResultMsg)
{
    UNUSED_VAR(ipResultMsg);

    hideProgressDialog();
}

//==================================================================================================
void CalibrationGuiBase::onLogMessageReceived(const rosgraph_msgs::Log::ConstPtr pLogMsg)
{
    //--- if name does not equal appTitle_, i.e. if the log message is from another process, return
    if (pLogMsg->name != appTitle_)
        return;

    //--- print log message in text box of calibration control window
    if (pCalibControlWindow_)
        pCalibControlWindow_->printLogMessage(pLogMsg);

    //--- strip new line of message in order to prepare for regex match
    std::string newlineStripedMsgStr = pLogMsg->msg;
    newlineStripedMsgStr.erase(std::remove(newlineStripedMsgStr.begin(),
                                           newlineStripedMsgStr.end(),
                                           '\n'),
                               newlineStripedMsgStr.cend());

    //--- if log level is higher than WARN and if log message begins with "[<appTitle_>...]",
    //--- additionally print message in QMessagePox
    if (pLogMsg->level >= rosgraph_msgs::Log::WARN &&
        std::regex_match(newlineStripedMsgStr, std::regex("(\\[" + appTitle_ + "(.*)\\])(.*)")))
    {
        //--- strip message from leading '[...]'
        QString strippedMsg = QString::fromStdString(
          pLogMsg->msg.substr(pLogMsg->msg.find_first_of(']') + 2));

        switch (pLogMsg->level)
        {
        default:
        case rosgraph_msgs::Log::WARN:
        {
            QMessageBox::information(pCalibControlWindow_.get(), pCalibControlWindow_->windowTitle(),
                                     strippedMsg);
        }
        break;

        case rosgraph_msgs::Log::ERROR:
        {
            QMessageBox::warning(pCalibControlWindow_.get(), pCalibControlWindow_->windowTitle(),
                                 strippedMsg);
        }
        break;

        case rosgraph_msgs::Log::FATAL:
        {
            QMessageBox::critical(pCalibControlWindow_.get(), pCalibControlWindow_->windowTitle(),
                                  strippedMsg);
        }
        break;
        }
    }
}

//==================================================================================================
bool CalibrationGuiBase::setupGuiElements()
{
    //--- get screen geometry to later place the individual windows
    screenGeometry_ = QApplication::primaryScreen()->availableGeometry();
    screenGeometry_.setHeight(screenGeometry_.height() - 100); // X11: availableGeometry also returns space which is reserved by window manager.

    //--- get height of titlebar
    titleBarHeight_ = QApplication::style()->pixelMetric(QStyle::PM_TitleBarHeight);

    //--- CalibrationControlWindow at the Top-Left Corner of Display
    pCalibControlWindow_ = std::make_shared<CalibrationControlWindow>();
    pCalibControlWindow_->setWindowTitle(QString::fromStdString(appTitle_.substr(1)));
    pCalibControlWindow_->move(0, 0);
    pCalibControlWindow_->setFixedSize((screenGeometry_.width() / 2) - 1,
                                       (screenGeometry_.height() / 2) - titleBarHeight_ - 1);
    pCalibControlWindow_->pbVisCalibrationPtr()->setEnabled((pNodeletLoader_ != nullptr));
    pCalibControlWindow_->show();

    //--- connect close signals
    connect(pCalibControlWindow_.get(), &CalibrationControlWindow::closed,
            this, &CalibrationGuiBase::closed);

    //--- connect push buttons to slots
    connect(pCalibControlWindow_->actionOpenCalibWsPtr(), &QAction::triggered,
            this, &CalibrationGuiBase::onActionOpenCalibWsTriggered);
    connect(pCalibControlWindow_->actionOpenRobotWsPtr(), &QAction::triggered,
            this, &CalibrationGuiBase::onActionOpenRobotWsTriggered);
    connect(pCalibControlWindow_->actionResetCalibrationPtr(), &QAction::triggered,
            this, &CalibrationGuiBase::onActionResetCalibTriggered);
    connect(pCalibControlWindow_->actionOpenPreferencesPtr(), &QAction::triggered,
            this, &CalibrationGuiBase::onActionPreferencesTriggered);
    connect(pCalibControlWindow_->pbCaptureTargetPtr(), &QPushButton::clicked,
            this, &CalibrationGuiBase::onCaptureTargetButtonClicked);
    connect(pCalibControlWindow_->pbFinalizeCalibrationPtr(), &QPushButton::clicked,
            this, &CalibrationGuiBase::onFinalizeCalibrationButtonClicked);
    connect(pCalibControlWindow_->pbRemoveObservationPtr(), &QPushButton::clicked,
            this, &CalibrationGuiBase::onRemoveObservationButtonClicked);
    connect(pCalibControlWindow_->pbVisCalibrationPtr(), &QPushButton::clicked,
            this, &CalibrationGuiBase::onVisualizeCalibrationButtonClicked);

    //--- Infinite progress dialog
    pProgressDialog_ = std::make_shared<QProgressDialog>(pCalibControlWindow_.get());
    pProgressDialog_->setWindowTitle("Please Wait!");
    pProgressDialog_->setCancelButton(nullptr);
    pProgressDialog_->setWindowModality(Qt::ApplicationModal);
    pProgressDialog_->setMinimumWidth(300);
    pProgressDialog_->setRange(0, 0); // set to infinite
    pProgressDialog_->setValue(0);

    return (pCalibControlWindow_ != nullptr);
}

//==================================================================================================
void CalibrationGuiBase::showProgressDialog(const QString& iBusyText)
{
    //--- update progress dialog
    if (pProgressDialog_)
    {
        pProgressDialog_->setLabelText(iBusyText);
        pProgressDialog_->show();
    }

    //--- update mouse cursor
    QApplication::setOverrideCursor(Qt::BusyCursor);
}

//==================================================================================================
void CalibrationGuiBase::getCalibrationMetaData()
{
    //--- get calibration meta data
    ros::ServiceClient metaDataClient =
      nh_.serviceClient<CalibrationMetaData>(calibratorNodeletName_ +
                                             "/" + REQUEST_META_DATA_SRV_NAME);
    CalibrationMetaData srvMsg;
    if (!metaDataClient.call(srvMsg))
    {
        ROS_ERROR_ONCE("[%s] Failed to get calibration meta data. "
                       "Check if calibration nodelet is initialized!",
                       guiNodeName_.c_str());
    }

    calibrationMetaData_ = srvMsg.response;

    //--- if calibration metadata is complete stop timer
    if (calibrationMetaData_.isComplete)
    {
        calibMetaDataTimer_.stop();
        initializeGuiContents();
    }
}

//==================================================================================================
void CalibrationGuiBase::onActionOpenCalibWsTriggered() const
{
    QDesktopServices::openUrl(
      QUrl(QString("file://%1")
             .arg(QString::fromStdString(calibrationMetaData_.calib_ws_path))));
}

//==================================================================================================
void CalibrationGuiBase::onActionOpenRobotWsTriggered() const
{
    QDesktopServices::openUrl(
      QUrl(QString("file://%1")
             .arg(QString::fromStdString(calibrationMetaData_.robot_ws_path))));
}

//==================================================================================================
void CalibrationGuiBase::onActionPreferencesTriggered()
{
    if (!pRqtReconfigureProcess_)
    {
        pRqtReconfigureProcess_ = std::make_shared<QProcess>(this);
        pRqtReconfigureProcess_->setProgram("rosrun");
        pRqtReconfigureProcess_->setArguments({"rqt_reconfigure", "rqt_reconfigure"});
    }

    if (pRqtReconfigureProcess_ && pRqtReconfigureProcess_->state() == QProcess::NotRunning)
    {
        //--- Start process
        pRqtReconfigureProcess_->start();
    }
    if (pRqtReconfigureProcess_ && pRqtReconfigureProcess_->state() == QProcess::Running)
    {
        //--- Kill process to bring window to the front

        pRqtReconfigureProcess_->kill();

        bool isFinished = false;
        do
        {
            QCoreApplication::processEvents();
            isFinished = pRqtReconfigureProcess_->waitForFinished(500);
        } while (!isFinished);

        pRqtReconfigureProcess_->start();
    }
}

//==================================================================================================
void CalibrationGuiBase::onActionResetCalibTriggered()
{
    // Function to do service call
    auto doServiceCall = [&]()
    {
        ResetCalibration srvMsg;

        //--- call service to reset calibration
        ros::ServiceClient resetServiceClient =
          nh_.serviceClient<ResetCalibration>(guidanceNodeletName_ +
                                              "/" + RESET_SRV_NAME);
        bool isSuccessful = resetServiceClient.call(srvMsg);

        resetServiceClient =
          nh_.serviceClient<ResetCalibration>(calibratorNodeletName_ +
                                              "/" + RESET_SRV_NAME);
        isSuccessful &= resetServiceClient.call(srvMsg);

        if (!isSuccessful)
        {
            ROS_ERROR("[%s] Failed to trigger reset of calibration. ",
                      guiNodeName_.c_str());
        }
    };

    //--- show progress dialog
    showProgressDialog("Reset calibration ...");

    //--- initialize visualizer asynchronously
    auto initFuture = std::async(doServiceCall);

    //--- while future is not ready, process QEvents in order for the progress dialog not to freeze
    while (initFuture.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
        QCoreApplication::processEvents();

    pCalibControlWindow_->clearLogMessages();

    hideProgressDialog();
}

//==================================================================================================
void CalibrationGuiBase::onActionImportObservationsTriggered()
{
    //--- get sensor name from the data of the action
    QString sensorName = "";
    QAction* pAction   = dynamic_cast<QAction*>(sender());
    if (pAction)
        sensorName = pAction->data().toString();
    else
        return;

    //--- open file dialog to choose directory
    QString directoryPath = QFileDialog::getExistingDirectory(
      pCalibControlWindow_.get(),
      QString("Open directory to import observations to '%1'...").arg(sensorName),
      QString::fromStdString(calibrationMetaData_.calib_ws_path));

    if (directoryPath.isEmpty())
        return;

    //--- get contents
    QDir observationParentDir(directoryPath);
    QStringList subdirList =
      observationParentDir.entryList(QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot);

    //--- check if subdirs exist and the iteration index starts of with 1
    if (subdirList.empty() ||
        !QFileInfo::exists(observationParentDir.absolutePath() + QDir::separator() +
                           "1" + QDir::separator() +
                           sensorName + QString::fromStdString(OBSERVATIONS_FILE_SUFFIX)))
    {
        QMessageBox::critical(pCalibControlWindow_.get(), "No observations found",
                              QString("There has been an error in trying to import observations from "
                                      "directory '%1'. Make sure that the given directory holds a number of "
                                      "subdirectories with the calibration iteration number "
                                      "(starting with '1') as directory name, e.g. 1, 2, 3, ... . "
                                      "And each subdirectory should hold a file named "
                                      "<sensor_name>%2. Take a look at the "
                                      "saved observations in the calibration workspace.")
                                .arg(directoryPath)
                                .arg(QString::fromStdString(OBSERVATIONS_FILE_SUFFIX)));
    }
    else
    {
        // service message to import marker observations
        ImportMarkerObservations importMarkerObsSrvMsg;

        //--- read observation data from individual directories
        for (int i = 1; i <= subdirList.size(); i++)
        {
            // path to file holding the corner observation of the specific calibration iteration
            fs::path observationFilePath = fs::path(
              (observationParentDir.absolutePath() + QDir::separator() +
               QString::number(i) + QDir::separator() +
               sensorName + QString::fromStdString(OBSERVATIONS_FILE_SUFFIX))
                .toStdString());

            //--- if file does not exist, continue
            if (!fs::exists(observationFilePath))
                continue;

            //--- read marker observations from file
            std::vector<uint> markerIds;
            std::vector<std::array<cv::Point3f, 4>> markerCorners;
            DataProcessor3d::readMarkerObservationsFromFile(observationFilePath,
                                                            markerIds, markerCorners);

            //--- add marker observations to service message
            MarkerObservations markerObsMsg;
            for (uint j = 0; j < markerIds.size(); j++)
            {
                markerObsMsg.marker_ids.push_back(markerIds[j]);

                markerObsMsg.marker_top_left_point.push_back(geometry_msgs::Point());
                markerObsMsg.marker_top_left_point.back().x = markerCorners[j][0].x;
                markerObsMsg.marker_top_left_point.back().y = markerCorners[j][0].y;
                markerObsMsg.marker_top_left_point.back().z = markerCorners[j][0].z;
            }

            importMarkerObsSrvMsg.request.observation_list.push_back(markerObsMsg);
        }

        // Function to do service call
        auto doServiceCall = [&](ImportMarkerObservations srvMsg)
        {
            //--- call service to import observation
            ros::ServiceClient importObservationsClient =
              nh_.serviceClient<ImportMarkerObservations>(calibratorNodeletName_ +
                                                          "/" + sensorName.toStdString() +
                                                          "/" + IMPORT_MARKER_OBS_SRV_NAME);

            if (!importObservationsClient.call(srvMsg))
            {
                ROS_ERROR("[%s] Failed to import observations. ",
                          guiNodeName_.c_str());
            }
        };

        //--- show progress dialog
        showProgressDialog(QString("Import observations to '%1' ...").arg(sensorName));

        //--- initialize visualizer asynchronously
        auto initFuture = std::async(doServiceCall, importMarkerObsSrvMsg);

        //--- while future is not ready, process QEvents in order for the progress dialog not to freeze
        while (initFuture.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
            QCoreApplication::processEvents();

        hideProgressDialog();
    }
}

//==================================================================================================
void CalibrationGuiBase::onCaptureTargetButtonClicked()
{
    // Function to do service call
    auto doServiceCall = [&]()
    {
        //--- call service to capture target
        ros::ServiceClient captureTargetClient =
          nh_.serviceClient<CaptureCalibTarget>(calibratorNodeletName_ +
                                                "/" + CAPTURE_TARGET_SRV_NAME);
        CaptureCalibTarget srvMsg;
        if (!captureTargetClient.call(srvMsg))
        {
            ROS_ERROR("[%s] Failed to trigger capturing of calibration target. ",
                      guiNodeName_.c_str());
        }
    };

    //--- show progress dialog
    showProgressDialog("Capturing target ...");

    //--- initialize visualizer asynchronously
    auto initFuture = std::async(doServiceCall);

    //--- while future is not ready, process QEvents in order for the progress dialog not to freeze
    while (initFuture.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
        QCoreApplication::processEvents();
}

//==================================================================================================
void CalibrationGuiBase::onFinalizeCalibrationButtonClicked()
{

    // Function to do service call
    auto doServiceCall = [&]()
    {
        //--- call service to finalize calibration
        ros::ServiceClient finalizeCalibrationClient =
          nh_.serviceClient<FinalizeCalibration>(calibratorNodeletName_ +
                                                 "/" + FINALIZE_CALIBRATION_SRV_NAME);
        FinalizeCalibration srvMsg;
        if (!finalizeCalibrationClient.call(srvMsg))
        {
            ROS_ERROR("[%s] Failed to trigger calibration finalization. ",
                      guiNodeName_.c_str());
        }
        else if (!srvMsg.response.isAccepted)
        {
            ROS_ERROR("[%s] Failed to finalize calibration. %s",
                      guiNodeName_.c_str(), srvMsg.response.msg.c_str());
        }
    };

    //--- show progress dialog
    showProgressDialog("Finalizing calibration ...");

    //--- initialize visualizer asynchronously
    auto initFuture = std::async(doServiceCall);

    //--- while future is not ready, process QEvents in order for the progress dialog not to freeze
    while (initFuture.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
        QCoreApplication::processEvents();
}

//==================================================================================================
void CalibrationGuiBase::onRemoveObservationButtonClicked()
{
    // Function to do service call
    auto doServiceCall = [&]()
    {
        //--- call service to remove observation
        ros::ServiceClient removeObservationClient =
          nh_.serviceClient<RemoveLastObservation>(calibratorNodeletName_ +
                                                   "/" + REMOVE_OBSERVATION_SRV_NAME);
        FinalizeCalibration srvMsg;
        if (!removeObservationClient.call(srvMsg))
        {
            ROS_ERROR("[%s] Failed to trigger removal of last observation. ",
                      guiNodeName_.c_str());
        }
    };

    //--- show progress dialog
    showProgressDialog("Removing last observation ...");

    //--- initialize visualizer asynchronously
    auto initFuture = std::async(doServiceCall);

    //--- while future is not ready, process QEvents in order for the progress dialog not to freeze
    while (initFuture.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
        QCoreApplication::processEvents();

    hideProgressDialog();
}

//==================================================================================================
void CalibrationGuiBase::onVisualizeCalibrationButtonClicked()
{
    loadVisualizer();
}

} // namespace multisensor_calibration