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

#include "../../include/multisensor_calibration/visualizers/PointCloud2ImageNodelet.h"

// Std
#include <cmath>
#include <iostream>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/LaserScan.h>

// PCL
#include <pcl/conversions.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/range_image/range_image_planar.h>

// OpenCV
#include <opencv2/calib3d.hpp>

// multisensor_calibration
#include "../../include/multisensor_calibration/common/utils.hpp"
#define NO_QT
#include "../../include/multisensor_calibration/common/lib3D/core/visualization.hpp"

namespace multisensor_calibration
{

static const std::string INPUT_CLOUD_TOPIC_NAME = "pointcloud";
static const std::string INPUT_IMAGE_TOPIC_NAME = "image";

static const std::string OUTPUT_TOPIC_NAME = "fused_image";

static float DEFAULT_MIN_DEPTH = 0.1f;
static float DEFAULT_MAX_DEPTH = 20.f;

static int PROJECTED_PIXEL_RADIUS = 3;

//==================================================================================================
PointCloud2ImageNodelet::PointCloud2ImageNodelet() :
  Nodelet(),
  isInitialized_(false),
  pImgCloudApproxSync_(nullptr),
  pImgCloudExactSync_(nullptr),
  imageState_(STR_2_IMG_STATE.at(DEFAULT_IMG_STATE_STR)),
  cameraNamespace_(""),
  minDepth_(DEFAULT_MIN_DEPTH),
  maxDepth_(DEFAULT_MAX_DEPTH),
  useTemporaryTransform_(false),
  syncQueueSize_(DEFAULT_SYNC_QUEUE_SIZE),
  useExactSync_(false)
{
    std::setlocale(LC_ALL, "en_US.UTF-8");
}

//==================================================================================================
PointCloud2ImageNodelet::~PointCloud2ImageNodelet()
{
    //--- reset pointers message filters before rest of the class in order to avoid seg fault during
    //--- disconnection of callbacks.
    pImgCloudApproxSync_.reset();
    pImgCloudExactSync_.reset();
}

//==================================================================================================
void PointCloud2ImageNodelet::computeCameraPoseForFrustumCulling(
  const lib3d::Camera& iCamera,
  Eigen::Matrix4f& oFrustumCullingPose) const
{
    Eigen::Matrix4f eigenRtMatrix;
    cv::cv2eigen(cv::Mat(iCamera.extrinsics.getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF)),
                 eigenRtMatrix);
    Eigen::Matrix4f cam2robot; // see documentation of pcl::FrustumCulling
    cam2robot << 0, 0, 1, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    oFrustumCullingPose = eigenRtMatrix * cam2robot;
}

//==================================================================================================
void PointCloud2ImageNodelet::doFrustumCulling(
  const pcl::PointCloud<InputPointType>::Ptr& ipCloud,
  const lib3d::Camera& iCamera,
  const float& iMinDepth,
  const float& iMaxDepth,
  pcl::Indices& oCulledPointIndices) const
{
    pcl::FrustumCulling<InputPointType> frustumCulling; // object of frustum culling
    Eigen::Matrix4f frustumCullingCamPose;              // camera pose used for frustum culling
    computeCameraPoseForFrustumCulling(iCamera, frustumCullingCamPose);
    frustumCulling.setHorizontalFOV(static_cast<float>(iCamera.intrinsics.getHFov()) + 50);
    frustumCulling.setVerticalFOV(static_cast<float>(iCamera.intrinsics.getVFov()) + 50);
    frustumCulling.setCameraPose(frustumCullingCamPose);
    frustumCulling.setInputCloud(ipCloud);
    frustumCulling.setNearPlaneDistance(iMinDepth);
    frustumCulling.setFarPlaneDistance(iMaxDepth);
    frustumCulling.filter(oCulledPointIndices);
}

//==================================================================================================
void PointCloud2ImageNodelet::onSensorDataReceived(const InputImage_Message_T::ConstPtr& ipImgMsg,
                                                   const InputCloud_Message_T::ConstPtr ipCloudMsg)
{
    //--- check if nodelet is initialized
    if (!isInitialized_)
        return;

    //--- get camera image from message
    cv::Mat img;                 // object of camera image
    std::string imgFrameId = ""; // frame id of camera image
    if (ipImgMsg != nullptr)
        getImageFromMessage(ipImgMsg, img, imgFrameId);

    //--- get cloud data
    std::string cloudFrameId = ipCloudMsg->header.frame_id; // frame id of point cloud
    pcl::PointCloud<InputPointType>::Ptr pInputPointCloud(  // pointer to input point cloud
      new pcl::PointCloud<InputPointType>);
    pcl::fromROSMsg(*ipCloudMsg, *pInputPointCloud);

    //--- get transform between frameIds
    tf::StampedTransform cloudTransform;
    bool isSuccessful = true;
    if (!imgFrameId.empty())
        isSuccessful &= getTransformBetweenTfFrames(imgFrameId,
                                                    cloudFrameId,
                                                    cloudTransform);
    if (!isSuccessful)
        return;

    //--- set camera extrinsics from tf transform
    if (!imgFrameId.empty())
        utils::setCameraExtrinsicsFromTfTransform(
          cloudTransform, camera_.extrinsics);

    //--- perform view frustum culling for initial filtering of the point cloud
    pcl::Indices culledPntIndices;
    if (!imgFrameId.empty())
        doFrustumCulling(pInputPointCloud, camera_, minDepth_, maxDepth_, culledPntIndices);

    //--- loop over point cloud and create depth maps
    lib3d::DepthMap depthMap(img.size()); // depth map for image
#ifndef DEBUG
#pragma omp parallel shared(pInputPointCloud, culledPntIndices, depthMap, camera_)
#endif
    {
        //--- Project points that are only visible in vis image

#ifndef DEBUG
#pragma omp for
#endif
        for (std::vector<int>::const_iterator idxItr = culledPntIndices.begin();
             idxItr != culledPntIndices.end();
             ++idxItr)
        {
            projectCloudPointIntoDepthMap(pInputPointCloud->points[*idxItr],
                                          camera_, depthMap);
        }
    }

    //--- colorize depth maps and create fused images
    //--- invert colorization due to rgb instead of bgr
    cv::Mat fusedImage;
    float minVal = minDepth_;
    float maxVal = maxDepth_;
    if (!depthMap.empty())
        fusedImage = lib3d::colorizeRangeImg(depthMap, lib3d::COLORMAP_RAINBOW,
                                             true, &minVal, &maxVal,
                                             img, 1.f);

    //--- publish fused images
    if (!fusedImage.empty())
    {
        std_msgs::Header msgHeader = ipImgMsg->header;
        sensor_msgs::ImagePtr imgMsg =
          cv_bridge::CvImage(msgHeader, sensor_msgs::image_encodings::RGB8,
                             fusedImage)
            .toImageMsg();
        pub_.publish(imgMsg);
    }
}

//==================================================================================================
void PointCloud2ImageNodelet::getImageFromMessage(const sensor_msgs::ImageConstPtr& ipImgMsg,
                                                  cv::Mat& oImage,
                                                  std::string& oFrameId) const
{
    //--- get frame id from header
    oFrameId = ipImgMsg->header.frame_id;

    //--- get image from message
    cv_bridge::CvImageConstPtr pCvImg;
    try
    {
        pCvImg = cv_bridge::toCvShare(ipImgMsg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("[%s] cv_bridge::Exception: %s", __PRETTY_FUNCTION__, e.what());
        return;
    }

    //--- convert to RGB or simply copy to member variable
    if (pCvImg->encoding == "mono8")
        cv::cvtColor(pCvImg->image, oImage, CV_GRAY2RGB);
    else if (pCvImg->encoding == "bgr8")
        cv::cvtColor(pCvImg->image, oImage, CV_BGR2RGB);
    else
        pCvImg->image.copyTo(oImage);
}

//==================================================================================================
bool PointCloud2ImageNodelet::getTransformBetweenTfFrames(
  const std::string& iTargetFrame,
  const std::string& iSourceFrame,
  tf::StampedTransform& oTransform) const
{
    //--- if temporary transform is to be used, construct output from stamped transform,
    //--- else use tfListener_ to look up transform
    if (useTemporaryTransform_)
    {
        oTransform = tf::StampedTransform(temporaryTransform_, ros::Time::now(),
                                          iTargetFrame, iSourceFrame);
    }
    else
    {

        //--- get transform between cloudFrameId_ and imageFrameId_ from tfListener
        try
        {
            tfListener_.lookupTransform(iTargetFrame, iSourceFrame, ros::Time(0), oTransform);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("[%s] tf::TransformException: %s", nodeletName_.c_str(), ex.what());
            return false;
        }
    }

    return true;
}

//==================================================================================================
bool PointCloud2ImageNodelet::initializeCameraData(
  const std::string& iImgTopic,
  const EImageState& iImgState,
  const std::string& iCameraNamespace,
  lib3d::Camera& oCamera) const
{
    //--- construct camera info topic, if camera_namespace is empty construct from image topic
    std::string cameraInfoTopicName = "";
    if (iCameraNamespace.empty())
    {
        std::size_t pos     = iImgTopic.find_last_of("/");
        cameraInfoTopicName = iImgTopic.substr(0, pos) + "/camera_info";
    }
    else
    {
        //--- strip cameraNamespace_ from last "/"
        cameraInfoTopicName = iCameraNamespace;
        while (cameraInfoTopicName.back() == '/')
            cameraInfoTopicName.pop_back();
        cameraInfoTopicName += "/camera_info";
    }

    //--- get camera intrinsics from camera_info
    sensor_msgs::CameraInfoConstPtr camInfo =
      ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cameraInfoTopicName,
                                                          ros::Duration(20, 0));

    if (camInfo != nullptr)
    {
        utils::setCameraIntrinsicsFromCameraInfo(*camInfo, oCamera.intrinsics, iImgState);
    }
    else
    {
        ROS_WARN("[%s] Wait for message of camera_info topic has timed out. "
                 "Camera intrinsics will not be set! "
                 "Image topic: %s",
                 nodeletName_.c_str(), iImgTopic.c_str());

        return false;
    }

    return true;
}

//==================================================================================================
bool PointCloud2ImageNodelet::initializePublishers(ros::NodeHandle& ioNh)
{
    //--- advertise fused image
    image_transport::ImageTransport pImgTransp(ioNh);
    pub_ = pImgTransp.advertise(OUTPUT_TOPIC_NAME, 10);

    return true;
}

//==================================================================================================
bool PointCloud2ImageNodelet::initializeSubscribers(ros::NodeHandle& ioNh)
{
    //--- subscribe to topics

    image_transport::ImageTransport imgTransp(ioNh);
    imageSubsc_.subscribe(imgTransp, INPUT_IMAGE_TOPIC_NAME, 1);

    cloudSubsc_.subscribe(ioNh, INPUT_CLOUD_TOPIC_NAME, 1);

    //--- initialize synchronizers
    //--- choose synchronization model and message callback dependent on fusion mode
    if (useExactSync_)
    {
        pImgCloudExactSync_.reset(new message_filters::Synchronizer<ImgCloudExactSync>(
          ImgCloudExactSync(10), imageSubsc_, cloudSubsc_));
        pImgCloudExactSync_->registerCallback(
          boost::bind(&PointCloud2ImageNodelet::onSensorDataReceived, this, _1, _2));
    }
    else
    {
        pImgCloudApproxSync_.reset(new message_filters::Synchronizer<ImgCloudApproxSync>(
          ImgCloudApproxSync(syncQueueSize_), imageSubsc_, cloudSubsc_));
        pImgCloudApproxSync_->registerCallback(
          boost::bind(&PointCloud2ImageNodelet::onSensorDataReceived, this, _1, _2));
    }

    return true;
}

//==================================================================================================
void PointCloud2ImageNodelet::onInit()
{
#ifdef DEBUG
    ROS_DEBUG("%s", __PRETTY_FUNCTION__);
#endif

    //--- get global and private node handle
    nh_                  = getNodeHandle();
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    //--- set nodelet name
    nodeletName_ = Nodelet::getName();

    //--- read launch parameters
    isInitialized_ = readLaunchParameters(pnh);

    //--- initialize subscribers
    isInitialized_ &= initializeSubscribers(nh_);

    //--- initialized publishers
    isInitialized_ &= initializePublishers(pnh);

    //--- initialize camera info data
    isInitialized_ &= initializeCameraData(imageSubsc_.getTopic(), imageState_,
                                           cameraNamespace_, camera_);
}

//==================================================================================================
bool PointCloud2ImageNodelet::readLaunchParameters(const ros::NodeHandle& iNh)
{
    //--- read launch parameters
    std::string imageStateStr, tmpTransformStr;
    iNh.param<std::string>("image_state", imageStateStr, DEFAULT_IMG_STATE_STR);
    iNh.param<std::string>("camera_namespace", cameraNamespace_, std::string(""));
    iNh.param<float>("min_depth", minDepth_, DEFAULT_MIN_DEPTH);
    iNh.param<float>("max_depth", maxDepth_, DEFAULT_MAX_DEPTH);
    iNh.param<int>("sync_queue_size", syncQueueSize_, DEFAULT_SYNC_QUEUE_SIZE);
    iNh.param<bool>("use_exact_sync", useExactSync_, false);
    iNh.param<std::string>("temp_transform", tmpTransformStr, std::string(""));
    if (!tmpTransformStr.empty())
    {
        useTemporaryTransform_ = true;
    }

    //--- sanity check for params
    if (minDepth_ <= 0)
    {
        ROS_WARN("[%s] (min_depth <= 0). Setting min. depth to default: %f ",
                 nodeletName_.c_str(), DEFAULT_MIN_DEPTH);
        minDepth_ = DEFAULT_MIN_DEPTH;
    }
    if (maxDepth_ <= 0)
    {
        ROS_WARN("[%s] (max_depth <= 0). Setting max. depth to default: %f ",
                 nodeletName_.c_str(), DEFAULT_MAX_DEPTH);
        maxDepth_ = DEFAULT_MAX_DEPTH;
    }
    if (minDepth_ >= maxDepth_)
    {

        ROS_WARN("[%s] (min_depth >= max_depth). Setting min. and max depth to default: [%f, %f]",
                 nodeletName_.c_str(), DEFAULT_MIN_DEPTH, DEFAULT_MAX_DEPTH);
        minDepth_ = DEFAULT_MIN_DEPTH;
        maxDepth_ = DEFAULT_MAX_DEPTH;
    }
    if (syncQueueSize_ <= 0)
    {
        ROS_WARN("[%s] (sync_queue_size <= 0). Setting synchronization queue size to default: %i ",
                 nodeletName_.c_str(), DEFAULT_SYNC_QUEUE_SIZE);
        syncQueueSize_ = DEFAULT_SYNC_QUEUE_SIZE;
    }

    //--- get image state from list
    auto stateFindItr = STR_2_IMG_STATE.find(imageStateStr);
    if (stateFindItr != STR_2_IMG_STATE.end())
        imageState_ = stateFindItr->second;

    //--- get temporary transform from tmpTransformStr
    if (useTemporaryTransform_)
    {
        //--- split tmpTransformStr by space into list of floats
        std::vector<float> tmpTransformFltList = utils::splitStringToFloat(tmpTransformStr, ' ');

        //--- check if temp_transform is of correct size
        if (tmpTransformFltList.size() == 6)
        {
            ROS_INFO("[%s] Using temporary transform (XYZ | RPY): %f %f %f | %f %f %f",
                     nodeletName_.c_str(), tmpTransformFltList[0], tmpTransformFltList[1],
                     tmpTransformFltList[2], tmpTransformFltList[3], tmpTransformFltList[4],
                     tmpTransformFltList[5]);

            //--- construct temporary transform from XYZ RPY
            tf::Vector3 xyz = tf::Vector3(tmpTransformFltList[0],
                                          tmpTransformFltList[1],
                                          tmpTransformFltList[2]);
            tf::Quaternion orientation;
            orientation.setRPY(tmpTransformFltList[3],
                               tmpTransformFltList[4],
                               tmpTransformFltList[5]);
            temporaryTransform_.setOrigin(xyz);
            temporaryTransform_.setRotation(orientation);
            temporaryTransform_ = temporaryTransform_.inverse();
        }
        else
        {
            ROS_WARN("[%s] Wrong format of temp_transform. Please provide as \"X Y Z R P Y\". "
                     "Extracting transform from frame IDs instead.",
                     nodeletName_.c_str());
            useTemporaryTransform_ = false;
        }
    }

    return true;
}

//==================================================================================================
void PointCloud2ImageNodelet::projectCloudPointIntoDepthMap(const InputPointType& iCloudPnt,
                                                            const lib3d::Camera& iCamera,
                                                            lib3d::DepthMap& ioDepthMap) const
{
    //--- transform world lidar point to local point
    cv::Vec4d _worldPnt(iCloudPnt.x, iCloudPnt.y, iCloudPnt.z, 1.0);
    cv::Vec4d _localPnt = iCamera.extrinsics.getRTMatrix() * _worldPnt;

    //--- project local point to pixel and get depth from z coordinate
    cv::Point2i px = iCamera.projectLocal2Pixel(static_cast<float>(_localPnt(0)),
                                                static_cast<float>(_localPnt(1)),
                                                static_cast<float>(_localPnt(2)));
    float pntDepth = static_cast<float>(_localPnt(2));

    cv::Size depthMapSize = ioDepthMap.size();

    //--- check if pixel is within image and depth is positive
    if (px.x >= 0 && px.x < depthMapSize.width &&
        px.y >= 0 && px.y < depthMapSize.height)
    {
        //--- dilate pixel to size of 3x3
        for (int m = -PROJECTED_PIXEL_RADIUS; m <= PROJECTED_PIXEL_RADIUS; ++m)
        {
            for (int n = -PROJECTED_PIXEL_RADIUS; n <= PROJECTED_PIXEL_RADIUS; ++n)
            {
                //--- check for image boundaries
                cv::Point2i neighborPx = px + cv::Point2i(n, m);
                neighborPx.x           = std::min(std::max(0, neighborPx.x),
                                                  depthMapSize.width - 1);
                neighborPx.y           = std::min(std::max(0, neighborPx.y),
                                                  depthMapSize.height - 1);

                //--- if depth map already holds value at specified pixel, only assign depth if
                //--- new depth is smaller
                float currentDepthVal = ioDepthMap.valueAt(px);
                if (currentDepthVal == 0 || currentDepthVal > pntDepth)
                    ioDepthMap.valueAt(neighborPx) = pntDepth;
            }
        }
    }
}

} // namespace multisensor_calibration

// Export the class as a plugin using the PLUGINLIB_EXPORT_CLASS macro.
PLUGINLIB_EXPORT_CLASS(multisensor_calibration::PointCloud2ImageNodelet, nodelet::Nodelet)