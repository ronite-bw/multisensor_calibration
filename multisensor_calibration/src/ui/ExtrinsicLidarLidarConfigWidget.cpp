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

#include "../../include/multisensor_calibration/ui/ExtrinsicLidarLidarConfigWidget.h"

// ROS
#include <ros/master.h>
#include <ros/package.h>

// Qt
#include <QDebug>

// multisensor_calibration
#include "../../include/multisensor_calibration/config/Workspace.h"
#include "ui_ExtrinsicLidarLidarConfigWidget.h"

namespace multisensor_calibration
{

//==================================================================================================
ExtrinsicLidarLidarConfigWidget::ExtrinsicLidarLidarConfigWidget(QWidget* parent) :
  QWidget(parent),
  ui(new Ui::ExtrinsicLidarLidarConfigWidget)
{
    //--- set up UI
    ui->setupUi(this);
}

//==================================================================================================
ExtrinsicLidarLidarConfigWidget::~ExtrinsicLidarLidarConfigWidget()
{
    delete ui;
}

//==================================================================================================
void ExtrinsicLidarLidarConfigWidget::clearCalibrationOptions()
{
    sensorPairSettingsMap_.clear();

    ui->srcNameComboBox->clear();
    ui->srcCloudTopicComboBox->clear();
    ui->refNameComboBox->clear();
    ui->refCloudTopicComboBox->clear();
    ui->baseFrameComboBox->clear();
    ui->uprightFrameIdComboBox->clear();
}

//==================================================================================================
void ExtrinsicLidarLidarConfigWidget::setRobotWorkspaceFolderPath(const QString& iFolderPath)
{
    robotWorkspaceDir_.setPath(iFolderPath);

    //--- disconnect slots from combo box signals
    disconnect(ui->srcNameComboBox,
               static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
               this,
               &ExtrinsicLidarLidarConfigWidget::handleSelectedSensorsChanged);
    disconnect(ui->refNameComboBox,
               static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
               this,
               &ExtrinsicLidarLidarConfigWidget::handleSelectedSensorsChanged);

    clearCalibrationOptions();
    populateCalibrationOptions();
    setCalibrationOptionsFromSettings();

    //--- connects slots to combo box signals
    connect(ui->srcNameComboBox,
            static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this,
            &ExtrinsicLidarLidarConfigWidget::handleSelectedSensorsChanged);
    connect(ui->refNameComboBox,
            static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this,
            &ExtrinsicLidarLidarConfigWidget::handleSelectedSensorsChanged);
}

//==================================================================================================
void ExtrinsicLidarLidarConfigWidget::setSourceSensorName(const QString& iSensorName)
{
    ui->srcNameComboBox->setCurrentText(iSensorName);
    handleSelectedSensorsChanged();
}

//==================================================================================================
QString ExtrinsicLidarLidarConfigWidget::getSourceSensorName() const
{
    return ui->srcNameComboBox->currentText();
}

//==================================================================================================
void ExtrinsicLidarLidarConfigWidget::setReferenceSensorName(const QString& iSensorName)
{
    ui->refNameComboBox->setCurrentText(iSensorName);
    handleSelectedSensorsChanged();
}

//==================================================================================================
QString ExtrinsicLidarLidarConfigWidget::getReferenceSensorName() const
{
    return ui->refNameComboBox->currentText();
}

//==================================================================================================
std::unordered_map<std::string, bool>
ExtrinsicLidarLidarConfigWidget::getBoolTypedCalibrationOptions()
{
    return {
      {"align_ground_planes", ui->alignGroundPlanesGroupBox->isChecked()},
      {"use_initial_guess", ui->initialGuessCheckBox->isChecked()},
      {"save_observations", ui->observationsCheckBox->isChecked()},
      {"use_exact_sync", (ui->syncPolicyComboBox->currentIndex() == 0)}};
}

//==================================================================================================
std::unordered_map<std::string, double>
ExtrinsicLidarLidarConfigWidget::getDoubleTypedCalibrationOptions()
{
    return {};
}

//==================================================================================================
std::unordered_map<std::string, int>
ExtrinsicLidarLidarConfigWidget::getIntTypedCalibrationOptions()
{
    return {
      {"sync_queue_size", ui->queueSizeSpinBox->value()}};
}

//==================================================================================================
std::unordered_map<std::string, std::string>
ExtrinsicLidarLidarConfigWidget::getStringTypedCalibrationOptions()
{
    return {
      {"src_lidar_sensor_name", ui->srcNameComboBox->currentText().toStdString()},
      {"src_lidar_cloud_topic", ui->srcCloudTopicComboBox->currentText().toStdString()},
      {"ref_lidar_sensor_name", ui->refNameComboBox->currentText().toStdString()},
      {"ref_lidar_cloud_topic", ui->refCloudTopicComboBox->currentText().toStdString()},
      {"base_frame_id", (ui->baseFrameGroupBox->isChecked())
                          ? ui->baseFrameComboBox->currentText().toStdString()
                          : ""},
      {"upright_frame_id", ui->uprightFrameIdComboBox->currentText().toStdString()},
      {"target_config_file", ros::package::getPath("multisensor_calibration") + "/cfg/" +
                               ui->calibTargetFileLineEdit->text().toStdString()},
    };
}

//==================================================================================================
void ExtrinsicLidarLidarConfigWidget::addStrUniquelyToComboBox(
  QComboBox* pComboBox, QString strVal) const
{
    if (pComboBox->findText(strVal) == -1)
        pComboBox->addItem(strVal);
}

//==================================================================================================
void ExtrinsicLidarLidarConfigWidget::populateCalibrationOptions()
{
    populateComboBoxesFromAvailableTopics();

    populateComboBoxesFromAvailableTfs();

    // TODO: change when supporting other Calibration targets
    ui->calibTargetFileLineEdit->setText("TargetWithCirclesAndAruco.yaml");

    if (!robotWorkspaceDir_.exists())
        return;

    //--- populate from settings files
    //--- loop over subdirectories in root workspace dir and evaluate appropriate settings files
    for (QString entry : robotWorkspaceDir_.entryList(QDir::AllDirs | QDir::NoDotAndDotDot,
                                                      QDir::Name))
    {
        QString fullDirectoryPath = robotWorkspaceDir_.absolutePath() + QDir::separator() + entry;

        //--- if entry is an appropriate workspace folder, populate options from settings file
        if (ExtrinsicLidarLidarCalibWorkspace::isValid(fullDirectoryPath.toStdString()))
        {
            QString settingsFilePath = fullDirectoryPath + QDir::separator() +
                                       QString::fromStdString(SETTINGS_FILE_NAME);
            std::shared_ptr<QSettings> pSettings =
              std::make_shared<QSettings>(settingsFilePath, QSettings::Format::IniFormat);

            //--- source lidar options
            QString srcLidarName = pSettings->value("source_lidar/sensor_name", "").toString();
            if (!srcLidarName.isEmpty())
                addStrUniquelyToComboBox(ui->srcNameComboBox, srcLidarName);

            QString tmpStrVal = pSettings->value("source_lidar/cloud_topic", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->srcCloudTopicComboBox, tmpStrVal);

            //--- reference lidar options
            QString refLidarName = pSettings->value("reference_lidar/sensor_name", "").toString();
            if (!refLidarName.isEmpty())
                addStrUniquelyToComboBox(ui->refNameComboBox, refLidarName);

            tmpStrVal = pSettings->value("reference_lidar/cloud_topic", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->refCloudTopicComboBox, tmpStrVal);

            //--- calibration options
            tmpStrVal = pSettings->value("calibration/base_frame_id", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->baseFrameComboBox, tmpStrVal);

            tmpStrVal = pSettings->value("calibration/upright_frame_id", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->uprightFrameIdComboBox, tmpStrVal);

            if (!srcLidarName.isEmpty() && !refLidarName.isEmpty())
            {
                QString key = srcLidarName + "_" + refLidarName;
                key.replace(" ", "_");
                sensorPairSettingsMap_[key.toStdString()] = pSettings;
            }
        }
    }
}

//==================================================================================================
void ExtrinsicLidarLidarConfigWidget::populateComboBoxesFromAvailableTfs()
{
    //--- populate combo boxes from available tf
    std::vector<std::string> frameIds;
    tfListener_.getFrameStrings(frameIds);
    for (std::string id : frameIds)
    {
        addStrUniquelyToComboBox(ui->baseFrameComboBox, QString::fromStdString(id));
        addStrUniquelyToComboBox(ui->uprightFrameIdComboBox, QString::fromStdString(id));
    }
}

//==================================================================================================
void ExtrinsicLidarLidarConfigWidget::populateComboBoxesFromAvailableTopics()
{

    //--- populate combo boxes from available ros topics
    ros::master::V_TopicInfo topicInfos;
    ros::master::getTopics(topicInfos);
    for (ros::master::TopicInfo topicInfo : topicInfos)
    {
        if (topicInfo.datatype == "sensor_msgs/PointCloud2")
        {
            QString topicName = QString::fromStdString(topicInfo.name);
            addStrUniquelyToComboBox(ui->srcCloudTopicComboBox, topicName);
            addStrUniquelyToComboBox(ui->refCloudTopicComboBox, topicName);
        }
    }
}

//==================================================================================================
void ExtrinsicLidarLidarConfigWidget::setCalibrationOptionsFromSettings()
{
    //--- construct key to access settings map
    QString srcLidarName = ui->srcNameComboBox->currentText();
    QString refLidarName = ui->refNameComboBox->currentText();
    if (srcLidarName.isEmpty() || refLidarName.isEmpty())
        return;

    QString key = srcLidarName + "_" + refLidarName;
    key.replace(" ", "_");
    if (sensorPairSettingsMap_.find(key.toStdString()) == sensorPairSettingsMap_.end())
        return;

    // Pointer to settings object
    std::shared_ptr<QSettings> pSettings = sensorPairSettingsMap_[key.toStdString()];

    //--- fill ui elements of calibration options

    ui->srcCloudTopicComboBox->setCurrentText(
      pSettings->value("source_lidar/cloud_topic").toString());

    ui->refCloudTopicComboBox->setCurrentText(
      pSettings->value("reference_lidar/cloud_topic").toString());

    ui->alignGroundPlanesGroupBox->setChecked(
      pSettings->value("calibration/align_ground_planes").toBool());

    ui->uprightFrameIdComboBox->setCurrentText(
      pSettings->value("calibration/upright_frame_id").toString());

    QString baseFrameIdVal = pSettings->value("calibration/base_frame_id").toString();
    if (!baseFrameIdVal.isEmpty())
    {
        ui->baseFrameGroupBox->setChecked(true);
        ui->baseFrameComboBox->setCurrentText(baseFrameIdVal);
    }
    else
    {
        ui->baseFrameGroupBox->setChecked(false);
    }
    // TODO: change when supporting other Calibration targets
    // ui->calibTargetFileLineEdit->setText(
    //   pSettings->value("calibration/target_config_file").toString());

    ui->observationsCheckBox->setChecked(
      pSettings->value("calibration/save_observations").toBool());
    ui->initialGuessCheckBox->setChecked(
      pSettings->value("calibration/use_initial_guess").toBool());

    ui->syncPolicyComboBox->setCurrentIndex((pSettings->value("misc/use_exact_sync").toBool())
                                              ? 0
                                              : 1);
    ui->queueSizeSpinBox->setValue(pSettings->value("misc/sync_queue_size").toInt());
}

//==================================================================================================
void ExtrinsicLidarLidarConfigWidget::handleSelectedSensorsChanged()
{
    setCalibrationOptionsFromSettings();
}

} // namespace multisensor_calibration
