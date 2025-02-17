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

#include "../../include/multisensor_calibration/ui/ExtrinsicCameraReferenceConfigWidget.h"

// ROS
#include <ros/master.h>
#include <ros/package.h>

// Qt
#include <QDebug>

// multisensor_calibration
#include "../../include/multisensor_calibration/config/Workspace.h"
#include "ui_ExtrinsicCameraReferenceConfigWidget.h"

namespace multisensor_calibration
{

//==================================================================================================
ExtrinsicCameraReferenceConfigWidget::ExtrinsicCameraReferenceConfigWidget(QWidget* parent) :
  QWidget(parent),
  ui(new Ui::ExtrinsicCameraReferenceConfigWidget)
{
    //--- set up UI
    ui->setupUi(this);

    for (uint i = 0; i < IMG_STATE_2_STR.size(); ++i)
    {
        ui->imageStateComboBox->addItem(
          QString::fromStdString(IMG_STATE_2_STR.at(static_cast<EImageState>(i))));
    }
}

//==================================================================================================
ExtrinsicCameraReferenceConfigWidget::~ExtrinsicCameraReferenceConfigWidget()
{
    delete ui;
}

//==================================================================================================
void ExtrinsicCameraReferenceConfigWidget::clearCalibrationOptions()
{
    sensorPairSettingsMap_.clear();

    ui->cameraNameComboBox->clear();
    ui->imageTopicComboBox->clear();
    ui->infoTopicComboBox->clear();
    ui->rightCameraNameComboBox->clear();
    ui->rightInfoComboBox->clear();
    ui->rectSuffixLineEdit->setText("_rect");
    ui->refNameComboBox->clear();
    ui->refFrameIdComboBox->clear();
    ui->baseFrameComboBox->clear();
}

//==================================================================================================
void ExtrinsicCameraReferenceConfigWidget::setRobotWorkspaceFolderPath(const QString& iFolderPath)
{
    robotWorkspaceDir_.setPath(iFolderPath);

    //--- disconnect slots from combo box signals
    disconnect(ui->cameraNameComboBox,
               static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
               this,
               &ExtrinsicCameraReferenceConfigWidget::handleSelectedSensorsChanged);
    disconnect(ui->refNameComboBox,
               static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
               this,
               &ExtrinsicCameraReferenceConfigWidget::handleSelectedSensorsChanged);

    clearCalibrationOptions();
    populateCalibrationOptions();
    setCalibrationOptionsFromSettings();

    addStrUniquelyToComboBox(ui->refNameComboBox, "reference");

    //--- connects slots to combo box signals
    connect(ui->cameraNameComboBox,
            static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this,
            &ExtrinsicCameraReferenceConfigWidget::handleSelectedSensorsChanged);
    connect(ui->refNameComboBox,
            static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this,
            &ExtrinsicCameraReferenceConfigWidget::handleSelectedSensorsChanged);
}

//==================================================================================================
void ExtrinsicCameraReferenceConfigWidget::setSourceSensorName(const QString& iSensorName)
{
    ui->cameraNameComboBox->setCurrentText(iSensorName);
    handleSelectedSensorsChanged();
}

//==================================================================================================
QString ExtrinsicCameraReferenceConfigWidget::getSourceSensorName() const
{
    return ui->cameraNameComboBox->currentText();
}

//==================================================================================================
void ExtrinsicCameraReferenceConfigWidget::setReferenceName(const QString& iSensorName)
{
    ui->refNameComboBox->setCurrentText(iSensorName);
    handleSelectedSensorsChanged();
}

//==================================================================================================
QString ExtrinsicCameraReferenceConfigWidget::getReferenceName() const
{
    return ui->refNameComboBox->currentText();
}

//==================================================================================================
std::unordered_map<std::string, bool>
ExtrinsicCameraReferenceConfigWidget::getBoolTypedCalibrationOptions()
{
    return {
      {"is_stereo_camera", ui->isStereoGroupBox->isChecked()},
      {"save_observations", ui->observationsCheckBox->isChecked()}};
}

//==================================================================================================
std::unordered_map<std::string, double>
ExtrinsicCameraReferenceConfigWidget::getDoubleTypedCalibrationOptions()
{
    return {};
}

//==================================================================================================
std::unordered_map<std::string, int>
ExtrinsicCameraReferenceConfigWidget::getIntTypedCalibrationOptions()
{
    return {};
}

//==================================================================================================
std::unordered_map<std::string, std::string>
ExtrinsicCameraReferenceConfigWidget::getStringTypedCalibrationOptions()
{
    std::string path = ros::package::getPath("multisensor_calibration");

    return {
      {"camera_sensor_name", ui->cameraNameComboBox->currentText().toStdString()},
      {"camera_image_topic", ui->imageTopicComboBox->currentText().toStdString()},
      {"camera_info_topic", ui->infoTopicComboBox->currentText().toStdString()},
      {"image_state", ui->imageStateComboBox->currentText().toStdString()},
      {"right_camera_sensor_name", ui->rightCameraNameComboBox->currentText().toStdString()},
      {"right_camera_info_topic", ui->rightInfoComboBox->currentText().toStdString()},
      {"rect_suffix", ui->rectSuffixLineEdit->text().toStdString()},
      {"reference_name", ui->refNameComboBox->currentText().toStdString()},
      {"reference_frame_id", ui->refFrameIdComboBox->currentText().toStdString()},
      {"base_frame_id", (ui->baseFrameGroupBox->isChecked())
                          ? ui->baseFrameComboBox->currentText().toStdString()
                          : ""},
      {"target_config_file", ros::package::getPath("multisensor_calibration") + "/cfg/" +
                               ui->calibTargetFileLineEdit->text().toStdString()},
    };
}

//==================================================================================================
void ExtrinsicCameraReferenceConfigWidget::addStrUniquelyToComboBox(
  QComboBox* pComboBox, QString strVal) const
{
    if (pComboBox->findText(strVal) == -1)
        pComboBox->addItem(strVal);
}

//==================================================================================================
void ExtrinsicCameraReferenceConfigWidget::populateCalibrationOptions()
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
        if (ExtrinsicCameraReferenceCalibWorkspace::isValid(fullDirectoryPath.toStdString()))
        {
            QString settingsFilePath = fullDirectoryPath + QDir::separator() +
                                       QString::fromStdString(SETTINGS_FILE_NAME);
            std::shared_ptr<QSettings> pSettings =
              std::make_shared<QSettings>(settingsFilePath, QSettings::Format::IniFormat);

            //--- camera options
            QString camSensorName = pSettings->value("camera/sensor_name", "").toString();
            if (!camSensorName.isEmpty())
                addStrUniquelyToComboBox(ui->cameraNameComboBox, camSensorName);

            QString tmpStrVal = pSettings->value("camera/image_topic", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->imageTopicComboBox, tmpStrVal);

            tmpStrVal = pSettings->value("camera/info_topic", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->infoTopicComboBox, tmpStrVal);

            tmpStrVal = pSettings->value("camera/right_sensor_name", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->rightCameraNameComboBox, tmpStrVal);

            tmpStrVal = pSettings->value("camera/right_info_topic", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->rightInfoComboBox, tmpStrVal);

            //--- reference options
            QString refName = pSettings->value("reference/name", "").toString();
            if (!refName.isEmpty())
                addStrUniquelyToComboBox(ui->refNameComboBox, refName);

            QString refFrameId = pSettings->value("reference/frame_id", "").toString();
            if (!refFrameId.isEmpty())
                addStrUniquelyToComboBox(ui->refFrameIdComboBox, refFrameId);

            //--- calibration options
            tmpStrVal = pSettings->value("calibration/base_frame_id", "").toString();
            if (!tmpStrVal.isEmpty())
                addStrUniquelyToComboBox(ui->baseFrameComboBox, tmpStrVal);

            if (!camSensorName.isEmpty() && !refName.isEmpty())
            {
                QString key = camSensorName + "_" + refName;
                key.replace(" ", "_");
                sensorPairSettingsMap_[key.toStdString()] = pSettings;
            }
        }
    }
}

//==================================================================================================
void ExtrinsicCameraReferenceConfigWidget::populateComboBoxesFromAvailableTfs()
{
    //--- populate combo boxes from available tf
    std::vector<std::string> frameIds;
    tfListener_.getFrameStrings(frameIds);
    for (std::string id : frameIds)
    {
        addStrUniquelyToComboBox(ui->baseFrameComboBox, QString::fromStdString(id));
        addStrUniquelyToComboBox(ui->refFrameIdComboBox, QString::fromStdString(id));
    }
}

//==================================================================================================
void ExtrinsicCameraReferenceConfigWidget::populateComboBoxesFromAvailableTopics()
{
    //--- populate combo boxes from available ros topics
    ros::master::V_TopicInfo topicInfos;
    ros::master::getTopics(topicInfos);
    for (ros::master::TopicInfo topicInfo : topicInfos)
    {
        if (topicInfo.datatype == "sensor_msgs/Image")
        {
            QString topicName = QString::fromStdString(topicInfo.name);
            addStrUniquelyToComboBox(ui->imageTopicComboBox, topicName);
        }
        else if (topicInfo.datatype == "sensor_msgs/CameraInfo")
        {
            QString topicName = QString::fromStdString(topicInfo.name);
            addStrUniquelyToComboBox(ui->infoTopicComboBox, topicName);
            addStrUniquelyToComboBox(ui->rightInfoComboBox, topicName);
        }
    }
}

//==================================================================================================
void ExtrinsicCameraReferenceConfigWidget::setCalibrationOptionsFromSettings()
{
    //--- construct key to access settings map
    QString camSensorName = ui->cameraNameComboBox->currentText();
    QString refName       = ui->refNameComboBox->currentText();
    if (camSensorName.isEmpty() || refName.isEmpty())
        return;

    QString key = camSensorName + "_" + refName;
    key.replace(" ", "_");
    if (sensorPairSettingsMap_.find(key.toStdString()) == sensorPairSettingsMap_.end())
        return;

    // Pointer to settings object
    std::shared_ptr<QSettings> pSettings = sensorPairSettingsMap_[key.toStdString()];

    //--- fill ui elements of calibration options

    ui->imageTopicComboBox->setCurrentText(pSettings->value("camera/image_topic").toString());
    ui->infoTopicComboBox->setCurrentText(pSettings->value("camera/info_topic").toString());
    ui->imageStateComboBox->setCurrentIndex(pSettings->value("camera/image_state").toInt());
    ui->isStereoGroupBox->setChecked(pSettings->value("camera/is_stereo_camera").toBool());
    ui->rightCameraNameComboBox->setCurrentText(
      pSettings->value("camera/right_sensor_name").toString());
    ui->rightInfoComboBox->setCurrentText(
      pSettings->value("camera/right_info_topic").toString());
    ui->rectSuffixLineEdit->setText(pSettings->value("camera/rect_suffix", "_rect").toString());

    ui->refFrameIdComboBox->setCurrentText(pSettings->value("reference/frame_id").toString());

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
}

//==================================================================================================
void ExtrinsicCameraReferenceConfigWidget::handleSelectedSensorsChanged()
{
    setCalibrationOptionsFromSettings();
}

} // namespace multisensor_calibration
