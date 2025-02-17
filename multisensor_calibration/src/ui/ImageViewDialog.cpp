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

#include "../../include/multisensor_calibration/ui/ImageViewDialog.h"

// Qt
#include <QCloseEvent>
#include <QImage>
#include <QMessageBox>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

// multisensor_calibration
#include "../../include/multisensor_calibration/common/lib3D/core/visualization.hpp"
#include "ui_ViewDialog.h"

namespace multisensor_calibration
{

//==================================================================================================
ImageViewDialog::ImageViewDialog(QWidget* parent) :
  QDialog(parent),
  ui(new Ui::ViewDialog),
  pImageGraphicsView_(new QGraphicsView),
  pImageGraphicsScene_(new QGraphicsScene),
  pPixmapItem_(nullptr)
{
    ui->setupUi(this);
    ui->verticalLayout->addWidget(pImageGraphicsView_.get());

    pImageGraphicsScene_->setBackgroundBrush(Qt::black);
    pImageGraphicsView_->setScene(pImageGraphicsScene_.get());
}

//==================================================================================================
ImageViewDialog::~ImageViewDialog()
{
    delete ui;
    delete pPixmapItem_;
}

//==================================================================================================
void ImageViewDialog::imageMessageCallback(const InputImage_Message_T::ConstPtr& ipImgMsg)
{
    //--- get image from message
    cv_bridge::CvImageConstPtr pCvBridgeImg;
    try
    {
        pCvBridgeImg = cv_bridge::toCvShare(ipImgMsg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("[%s] Exception while trying to convert image message."
                  "\n\t> Possible cause: Subscription to raw (non-debayered) image stream."
                  "\n\t> cv_bridge::Exception: %s",
                  __PRETTY_FUNCTION__, e.what());
        return;
    }

    // Object of OpenCV camera image
    cv::Mat cvImage;

    //--- convert to bgr or simply copy to member variable
    if (pCvBridgeImg->encoding == "mono8")
        cv::cvtColor(pCvBridgeImg->image, cvImage, CV_GRAY2RGB);
    else if (pCvBridgeImg->encoding == "bgr8")
        cv::cvtColor(pCvBridgeImg->image, cvImage, CV_BGR2RGB);
    else
        pCvBridgeImg->image.copyTo(cvImage);

    //--- create qimage with shared memory
    if (!pPixmapItem_)
        pPixmapItem_ = pImageGraphicsScene_->addPixmap(
          QPixmap::fromImage(lib3d::cvMat2QImage_shared(cvImage)));
    else
        pPixmapItem_->setPixmap(QPixmap::fromImage(lib3d::cvMat2QImage_shared(cvImage)));
    pImageGraphicsView_->fitInView(pPixmapItem_, Qt::KeepAspectRatio);
}

//==================================================================================================
void ImageViewDialog::subscribeToImageTopic(ros::NodeHandle& ioNh, const std::string& iTopicName)
{
    image_transport::ImageTransport imgTransp(ioNh);
    imageSubsc_ =
      imgTransp.subscribe(iTopicName, 10,
                          boost::bind(&ImageViewDialog::imageMessageCallback, this, _1));
}

} // namespace multisensor_calibration
