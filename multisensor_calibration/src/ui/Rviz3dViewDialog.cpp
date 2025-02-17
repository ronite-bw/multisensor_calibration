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

#include "../../include/multisensor_calibration/ui/Rviz3dViewDialog.h"

// ROS
#include <rviz/view_manager.h>

// Qt
#include <QCloseEvent>
#include <QMessageBox>

// multisensor_calibration
#include "../../include/multisensor_calibration/common/common.h"
#include "ui_ViewDialog.h"

namespace multisensor_calibration
{

//==================================================================================================
Rviz3dViewDialog::Rviz3dViewDialog(QWidget* parent) :
  QDialog(parent),
  pUi_(new Ui::ViewDialog),
  isInitialized_(false),
  pRenderPanel_(nullptr),
  pVisManager_(nullptr),
  fixedReferenceFrame_(""),
  axisReferenceFrames_(),
  cornerCloudTopicNames_(),
  sensorCloudTopicNames_(),
  roisCloudTopicNames_(),
  distanceCloudTopicNames_(),
  targetCloudTopicNames_()
{
    pUi_->setupUi(this);
}

//==================================================================================================
Rviz3dViewDialog::~Rviz3dViewDialog()
{
    delete pUi_;
}

//==================================================================================================
bool Rviz3dViewDialog::addAxes(const std::string& iReferenceFrame)
{
    //--- if visualization manager is not null, create display
    if (pVisManager_)
    {
        rviz::Display* pAxesDisplay_ =
          pVisManager_->createDisplay(
            "rviz/Axes",
            "Axes " + QString::number(axisReferenceFrames_.size()),
            true);

        if (!iReferenceFrame.empty())
            pAxesDisplay_->subProp("Reference Frame")->setValue(QString::fromStdString(iReferenceFrame));
    }

    //--- if reference name not in list, add to list
    if (std::find(axisReferenceFrames_.begin(), axisReferenceFrames_.end(), iReferenceFrame) ==
        axisReferenceFrames_.end())
        axisReferenceFrames_.push_back(iReferenceFrame);

    return true;
}

//==================================================================================================
bool Rviz3dViewDialog::addGuidedPlacementBox(const std::string& iTopicName)
{
    //--- if visualization manager is not null, create display
    if (pVisManager_)
    {
        rviz::Display* pMarkerDisplay_ =
          pVisManager_->createDisplay(
            "rviz/Marker",
            "Guided Placement Box " + QString::number(placementBoxTopicNames_.size()),
            true);
        pMarkerDisplay_->subProp("Marker Topic")->setValue(QString::fromStdString(iTopicName));
    }

    //--- if topic name not in list, add to list
    if (std::find(placementBoxTopicNames_.begin(), placementBoxTopicNames_.end(), iTopicName) ==
        placementBoxTopicNames_.end())
        placementBoxTopicNames_.push_back(iTopicName);

    return true;
}

//==================================================================================================
bool Rviz3dViewDialog::addMarkerCornersCloud(const std::string& iTopicName)
{
    //--- if visualization manager is not null, create display
    if (pVisManager_)
    {
        rviz::Display* pCloudDisplay_ =
          pVisManager_->createDisplay(
            "rviz/PointCloud2",
            "Marker Corners Cloud " + QString::number(cornerCloudTopicNames_.size()),
            true);
        pCloudDisplay_->subProp("Topic")->setValue(QString::fromStdString(iTopicName));
        pCloudDisplay_->subProp("Use Fixed Frame")->setValue("true");
        pCloudDisplay_->subProp("Color Transformer")->setValue("Intensity");
        pCloudDisplay_->subProp("Color")->setValue("252; 255; 255");
        pCloudDisplay_->subProp("Style")->setValue("Points");
        pCloudDisplay_->subProp("Size (Pixels)")->setValue(16);
    }

    //--- if topic name not in list, add to list
    if (std::find(cornerCloudTopicNames_.begin(), cornerCloudTopicNames_.end(), iTopicName) ==
        cornerCloudTopicNames_.end())
        cornerCloudTopicNames_.push_back(iTopicName);

    return true;
}

//==================================================================================================
bool Rviz3dViewDialog::addRawSensorCloud(const std::string& iTopicName)
{
    //--- if visualization manager is not null, create display
    if (pVisManager_)
    {
        rviz::Display* pCloudDisplay_ =
          pVisManager_->createDisplay(
            "rviz/PointCloud2",
            "Raw Sensor Cloud " + QString::number(sensorCloudTopicNames_.size()),
            true);
        pCloudDisplay_->subProp("Topic")->setValue(QString::fromStdString(iTopicName));
        pCloudDisplay_->subProp("Use Fixed Frame")->setValue("true");
        pCloudDisplay_->subProp("Color Transformer")->setValue("FlatColor");
        pCloudDisplay_->subProp("Color")->setValue("255; 255; 255");
        pCloudDisplay_->subProp("Style")->setValue("Points");
        pCloudDisplay_->subProp("Size (Pixels)")->setValue(1);
    }

    //--- if topic name not in list, add to list
    if (std::find(sensorCloudTopicNames_.begin(), sensorCloudTopicNames_.end(), iTopicName) ==
        sensorCloudTopicNames_.end())
        sensorCloudTopicNames_.push_back(iTopicName);

    return true;
}

//==================================================================================================
bool Rviz3dViewDialog::addRegionsOfInterestCloud(const std::string& iTopicName)
{
    //--- if visualization manager is not null, create display
    if (pVisManager_)
    {
        rviz::Display* pCloudDisplay_ =
          pVisManager_->createDisplay(
            "rviz/PointCloud2",
            "Regions-of-Interest Cloud " + QString::number(roisCloudTopicNames_.size()),
            true);
        pCloudDisplay_->subProp("Topic")->setValue(QString::fromStdString(iTopicName));
        pCloudDisplay_->subProp("Use Fixed Frame")->setValue("true");
        pCloudDisplay_->subProp("Color Transformer")->setValue("Intensity");
        pCloudDisplay_->subProp("Color")->setValue("255; 255; 255");
        pCloudDisplay_->subProp("Style")->setValue("Points");
        pCloudDisplay_->subProp("Size (Pixels)")->setValue(3);
    }

    //--- if topic name not in list, add to list
    if (std::find(roisCloudTopicNames_.begin(), roisCloudTopicNames_.end(), iTopicName) ==
        roisCloudTopicNames_.end())
        roisCloudTopicNames_.push_back(iTopicName);

    return true;
}

//==================================================================================================
bool Rviz3dViewDialog::addPointWiseDistanceCloud(const std::string& iTopicName)
{
    //--- if visualization manager is not null, create display
    if (pVisManager_)
    {
        rviz::Display* pCloudDisplay_ =
          pVisManager_->createDisplay(
            "rviz/PointCloud2",
            "Point-Wise Distance Cloud " + QString::number(distanceCloudTopicNames_.size()),
            true);
        pCloudDisplay_->subProp("Topic")->setValue(QString::fromStdString(iTopicName));
        pCloudDisplay_->subProp("Use Fixed Frame")->setValue("true");
        pCloudDisplay_->subProp("Color Transformer")->setValue("Intensity");
        pCloudDisplay_->subProp("Color")->setValue("255; 255; 255");
        pCloudDisplay_->subProp("Style")->setValue("Points");
        pCloudDisplay_->subProp("Size (Pixels)")->setValue(3);
    }

    //--- if topic name not in list, add to list
    if (std::find(distanceCloudTopicNames_.begin(), distanceCloudTopicNames_.end(), iTopicName) ==
        distanceCloudTopicNames_.end())
        distanceCloudTopicNames_.push_back(iTopicName);

    return true;
}

//==================================================================================================
bool Rviz3dViewDialog::addCalibTargetCloud(const std::string& iTopicName)
{
    //--- if visualization manager is not null, create display
    if (pVisManager_)
    {
        rviz::Display* pCloudDisplay_ =
          pVisManager_->createDisplay(
            "rviz/PointCloud2",
            "Calibration Target Cloud " + QString::number(targetCloudTopicNames_.size()),
            true);
        pCloudDisplay_->subProp("Topic")->setValue(QString::fromStdString(iTopicName));
        pCloudDisplay_->subProp("Use Fixed Frame")->setValue("true");
        pCloudDisplay_->subProp("Color Transformer")->setValue("FlatColor");
        pCloudDisplay_->subProp("Color")->setValue("252; 233; 79");
        pCloudDisplay_->subProp("Style")->setValue("Points");
        pCloudDisplay_->subProp("Size (Pixels)")->setValue(5);
    }

    //--- if topic name not in list, add to list
    if (std::find(targetCloudTopicNames_.begin(), targetCloudTopicNames_.end(), iTopicName) ==
        targetCloudTopicNames_.end())
        targetCloudTopicNames_.push_back(iTopicName);

    return true;
}

//==================================================================================================
void Rviz3dViewDialog::closeEvent(QCloseEvent* closeEvent)
{
    if (isInitialized_ && pVisManager_ != nullptr)
    {
        pUi_->verticalLayout->removeWidget(pRenderPanel_.get());

        pRenderPanel_.reset();
        pVisManager_.reset();

        isInitialized_ = false;
    }

    QDialog::closeEvent(closeEvent);
}

//==================================================================================================
bool Rviz3dViewDialog::setFixedReferenceFrame(const std::string& iFrameId)
{
    if (pVisManager_)
        pVisManager_->setFixedFrame(QString::fromStdString(iFrameId));

    fixedReferenceFrame_ = iFrameId;

    return true;
}

//==================================================================================================
bool Rviz3dViewDialog::setView(const EViews& iView)
{
    if (pVisManager_)
    {
        switch (iView)
        {
        default:
        case ORBIT:
        {
            pVisManager_->getViewManager()->setCurrentViewControllerType("rviz/Orbit");
        }
        break;
        case TOP_DOWN:
            pVisManager_->getViewManager()->setCurrentViewControllerType("rviz/TopDownOrtho");
            break;
        }
    }

    return true;
}

//==================================================================================================
void Rviz3dViewDialog::showEvent(QShowEvent* showEvent)
{
    if (!isInitialized_)
        initRenderPanel();

    QDialog::showEvent(showEvent);
}

//==================================================================================================
void Rviz3dViewDialog::initRenderPanel()
{
    //--- create instances
    pRenderPanel_ = std::make_shared<rviz::RenderPanel>();
    pVisManager_  = std::make_shared<rviz::VisualizationManager>(pRenderPanel_.get());

    //--- add panel to layout
    pUi_->verticalLayout->addWidget(pRenderPanel_.get());

    //--- initialize panel and visualization manager
    pRenderPanel_->initialize(pVisManager_->getSceneManager(), pVisManager_.get());
    pVisManager_->initialize();
    pVisManager_->startUpdate();

    //--- setup content
    if (!fixedReferenceFrame_.empty())
        setFixedReferenceFrame(fixedReferenceFrame_);
    for (std::string frameId : axisReferenceFrames_)
        addAxes(frameId);
    for (std::string topicName : cornerCloudTopicNames_)
        addMarkerCornersCloud(topicName);
    for (std::string topicName : sensorCloudTopicNames_)
        addRawSensorCloud(topicName);
    for (std::string topicName : roisCloudTopicNames_)
        addRegionsOfInterestCloud(topicName);
    for (std::string topicName : distanceCloudTopicNames_)
        addPointWiseDistanceCloud(topicName);
    for (std::string topicName : targetCloudTopicNames_)
        addCalibTargetCloud(topicName);

    isInitialized_ = true;
}

} // namespace multisensor_calibration
