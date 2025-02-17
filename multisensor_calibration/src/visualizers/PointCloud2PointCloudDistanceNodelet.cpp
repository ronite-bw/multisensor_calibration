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

#include "../../include/multisensor_calibration/visualizers/PointCloud2PointCloudDistanceNodelet.h"

// Std
#include <cmath>
#include <iostream>

// ROS
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/LaserScan.h>

// PCL
#include <pcl/PolygonMesh.h>
#include <pcl/common/centroid.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/gp3.h>
#include <pcl_ros/transforms.h>

// OpenCV
#include <opencv2/core.hpp>

// multisensor_calibration
#include "../../include/multisensor_calibration/common/common.h"
#include "../../include/multisensor_calibration/common/utils.hpp"

namespace multisensor_calibration
{

//--- TOPIC NAMES
static const std::string INPUT_TOPIC_NAME_PREFIX   = "cloud";
static const std::string OUTPUT_CLOUD_TOPIC_SUFFIX = "enhanced";

//--- DEFAULT VALUES
static int DEFAULT_NUMBER_CLOUDS    = 2;
static int DEFAULT_PROCESSING_RATE  = 1;
static int DEFAULT_DISTANCE_MEASURE = static_cast<int>(
  PointCloud2PointCloudDistanceNodelet::POINT_2_POINT);
static int DEFAULT_NN               = 5;
static float DEFAULT_MAX_DISTANCE   = 5.f;
static float DEFAULT_CLAMP_DISTANCE = FLT_MAX;

static std::vector<std::string> distanceMeasureStrList = {"POINT_2_POINT",
                                                          "POINT_2_SURFACE"};

//==================================================================================================
PointCloud2PointCloudDistanceNodelet::PointCloud2PointCloudDistanceNodelet() :
  useTemporaryTransform_(false)
{
    std::setlocale(LC_ALL, "en_US.UTF-8");
}

//==================================================================================================
PointCloud2PointCloudDistanceNodelet::~PointCloud2PointCloudDistanceNodelet()
{
}

//==================================================================================================
void PointCloud2PointCloudDistanceNodelet::onInit()
{
#ifdef DEBUG_BUILD
    ROS_INFO("%s", __PRETTY_FUNCTION__);
#endif

    //--- get global and private node handle
    nh_                  = getNodeHandle();
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    //--- read launch parameters
    int distanceMeasureInt, processingRate;
    std::string tmpTransformStr;
    pnh.param<int>("number_of_clouds", nClouds_, DEFAULT_NUMBER_CLOUDS);
    pnh.param<int>("processing_rate", processingRate, DEFAULT_PROCESSING_RATE);
    pnh.param<int>("distance_measure", distanceMeasureInt, DEFAULT_DISTANCE_MEASURE);
    pnh.param<int>("num_nearest_neighbors", nNearestNeighbors_, DEFAULT_NN);
    pnh.param<float>("max_distance", maxDistance_, DEFAULT_MAX_DISTANCE);
    pnh.param<float>("clamp_distance_threshold", clampDistanceThreshold_, DEFAULT_CLAMP_DISTANCE);
    pnh.param<std::string>("temp_transform", tmpTransformStr, std::string(""));
    if (!tmpTransformStr.empty() && nClouds_ == 2)
    {
        useTemporaryTransform_ = true;
    }

    //--- sanity check for params
    if (nClouds_ < 2)
    {
        ROS_WARN("(number_of_clouds < 2). Setting number clouds to default: %i ",
                 DEFAULT_NUMBER_CLOUDS);
        nClouds_ = DEFAULT_NUMBER_CLOUDS;
    }
    if (processingRate <= 0)
    {
        ROS_WARN("(processing_rate <= 0). Setting processing rate to default: %i Hz",
                 DEFAULT_PROCESSING_RATE);
        processingRate = DEFAULT_PROCESSING_RATE;
    }
    if (distanceMeasureInt < 0 || distanceMeasureInt > 1)
    {
        ROS_WARN("(distance_measure < 0 || distance_measure > 1). "
                 "Setting number distance measure to default: %s ",
                 distanceMeasureStrList[DEFAULT_DISTANCE_MEASURE].c_str());
        distanceMeasureInt = DEFAULT_DISTANCE_MEASURE;
    }
    if (nNearestNeighbors_ <= 0)
    {
        ROS_WARN("(num_nearest_neighbors <= 0). Setting number nearest neighbors to default: %i ",
                 DEFAULT_NN);
        nNearestNeighbors_ = DEFAULT_NN;
    }
    if (maxDistance_ <= 0)
    {
        ROS_WARN("(max_distance <= 0). Setting max. distance to default: %f ",
                 DEFAULT_MAX_DISTANCE);
        maxDistance_ = DEFAULT_MAX_DISTANCE;
    }
    if (clampDistanceThreshold_ < maxDistance_)
    {
        ROS_WARN("(clamp_distance_threshold < max_distance). Setting clamp. distance threshold to "
                 "max. distance: %f ",
                 maxDistance_);
        clampDistanceThreshold_ = maxDistance_;
    }

    //--- get temporary transform from tmpTransformStr
    if (useTemporaryTransform_)
    {
        //--- split tmpTransformStr by space into list of floats
        std::vector<float> tmpTransformFltList = utils::splitStringToFloat(tmpTransformStr, ' ');

        //--- check if temp_transform is of correct size
        if (tmpTransformFltList.size() == 6)
        {
            ROS_INFO("Using temporary transform (XYZ | RPY) "
                     "between cloud_1 (child) and cloud_0 (parent): %f %f %f | %f %f %f",
                     tmpTransformFltList[0], tmpTransformFltList[1],
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
        }
        else
        {
            ROS_WARN("Wrong format of temp_transform. Please provide as \"X Y Z R P Y\". "
                     "Extracting transform from frame IDs instead.");
            useTemporaryTransform_ = false;
        }
    }

    //--- initialize subscriber and register callback
    cloudSubscrs_ = std::vector<ros::Subscriber>(nClouds_);
    for (int i = 0; i < nClouds_; ++i)
    {
        std::string topicName = INPUT_TOPIC_NAME_PREFIX + "_" + std::to_string(i);
        cloudSubscrs_[i]      = nh_.subscribe<PointCloud>(
          topicName, 10,
          boost::bind(&PointCloud2PointCloudDistanceNodelet::cloudMessageCallback, this, _1, i));
    }

    //--- initialize publishers
    pubs_ = std::vector<ros::Publisher>(nClouds_);
    for (int i = 0; i < nClouds_; ++i)
    {
        std::string topicName = INPUT_TOPIC_NAME_PREFIX + "_" + std::to_string(i) + "_" +
                                OUTPUT_CLOUD_TOPIC_SUFFIX;
        pubs_[i] = pnh.advertise<PointCloud>(topicName, 10);
    }

    //--- initialize list of point clouds and the metadata
    pPointClouds_ = std::vector<pcl::PointCloud<InputPointType>::Ptr>(nClouds_, nullptr);
    // pKdTrees_           = std::vector<pcl::search::KdTree<InputPointType>::Ptr>(nClouds_);
    // pCloudNormals_      = std::vector<pcl::PointCloud<pcl::Normal>::Ptr>(nClouds_);
    pointCloudMsgHeaders_ = std::vector<std_msgs::Header>(nClouds_);

    //--- initialize processing timer (no autostart)
    processingTimer_ = nh_.createTimer(
      ros::Duration(1.0 / processingRate),
      boost::bind(&PointCloud2PointCloudDistanceNodelet::processCloudsCallback, this, _1),
      false, false);

    //--- select appropriate function to calculate the distance measure
    distanceMeasure_ = static_cast<EDistanceMeasure>(distanceMeasureInt);
    switch (distanceMeasure_)
    {
    case POINT_2_POINT:
        calculateDistanceFn_ = &calculatePoint2PointDistance;
        break;

    default:
    case POINT_2_SURFACE:
        calculateDistanceFn_ = &calculatePoint2SurfaceDistance;
        break;
    }
}

//==================================================================================================
void PointCloud2PointCloudDistanceNodelet::cloudMessageCallback(
  const InputCloud_Message_T::ConstPtr& ipCloudMsg, const int idx)
{
#ifdef DEBUG_BUILD
    ROS_INFO("%s | idx: %i | %i-%i", __PRETTY_FUNCTION__, idx,
             ipCloudMsg->header.stamp.sec, ipCloudMsg->header.stamp.nsec);
#endif

    //--- lock mutex
    pointCloudMutex_.lock();

    //--- copy point cloud into vector
    pPointClouds_[idx].reset(new pcl::PointCloud<InputPointType>);
    pcl::fromROSMsg(*ipCloudMsg, *pPointClouds_[idx]);

    //--- store message headers
    pointCloudMsgHeaders_[idx] = ipCloudMsg->header;

    //--- if processing timer is not yet started, check if data is available for all clouds and
    //--- start timer if so
    if (!processingTimer_.hasStarted())
    {
        bool isCloudDataAvailable = true;
        for (pcl::PointCloud<InputPointType>::Ptr pCloud : pPointClouds_)
        {
            if (pCloud == nullptr)
            {
                isCloudDataAvailable = false;
                break;
            }
        }
        if (isCloudDataAvailable)
            processingTimer_.start();
    }

    //--- unlock mutex
    pointCloudMutex_.unlock();
}

//==================================================================================================
void PointCloud2PointCloudDistanceNodelet::processCloudsCallback(
  const ros::TimerEvent& e)
{
    UNUSED_VAR(e)

#ifdef DEBUG_BUILD
    ROS_INFO("%s", __PRETTY_FUNCTION__);
#endif

    //--- lock mutex
    pointCloudMutex_.lock();

    //--- loop over point clouds and do distance calculation to each of the other point clouds
    //--- store the minimum of all distances inside the intensity field
    for (size_t s = 0; s < pPointClouds_.size(); ++s)
    {
        // pointer to source point cloud for which the distance is to be computed
        pcl::PointCloud<InputPointType>::Ptr pSourceXyzCloud = pPointClouds_[s];

        //--- initialize output cloud

        // source point cloud enhanced with the distance in the intensity field
        pcl::PointCloud<pcl::PointXYZI>::Ptr pEnhancedSourceXyziCloud(
          new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*pSourceXyzCloud, *pEnhancedSourceXyziCloud);

        // point indices that exceed clamp distance threshold
        pcl::IndicesPtr pClampedPointIndices(new pcl::Indices);

        // point-wise winner-takes-it-all distance
        std::vector<float> wtaDistance = std::vector<float>(pSourceXyzCloud->size(), FLT_MAX);

        for (size_t t = 0; t < pPointClouds_.size(); ++t)
        {
            //--- skip if target (t) equals to source (s)
            if (t == s)
                continue;

            //--- get transform from targetCloud to sourceCloud with tfListener
            //--- since the source cloud is to be enhanced with distance information, the target
            //--- cloud should be transformed into the coordinate system of the source cloud. In
            //--- this way, the source cloud does not need to be transformed back prior to
            //--- publishment.
            tf::StampedTransform transform;

            //--- if temporary transform is to be used, construct output from stamped transform,
            //--- else use tfListener_ to look up transform
            //--- temporary transform is only available if two clouds are used
            if (useTemporaryTransform_)
            {
                //--- if t is greater than s, i.e. if cloud2 to cloud1 is considered, use inverse
                if (s < t)
                    transform = tf::StampedTransform(temporaryTransform_, ros::Time::now(),
                                                     pointCloudMsgHeaders_[s].frame_id,
                                                     pointCloudMsgHeaders_[t].frame_id);
                else
                    transform = tf::StampedTransform(temporaryTransform_.inverse(),
                                                     ros::Time::now(),
                                                     pointCloudMsgHeaders_[s].frame_id,
                                                     pointCloudMsgHeaders_[t].frame_id);
            }
            else
            {
                try
                {
                    tfListener_.lookupTransform(pointCloudMsgHeaders_[s].frame_id,
                                                pointCloudMsgHeaders_[t].frame_id,
                                                ros::Time(0), transform);
                }
                catch (tf::TransformException& ex)
                {
                    ROS_ERROR("tf::TransformException: %s", ex.what());
                    return;
                }
            }

            // target point cloud transformed into frame of source point cloud
            pcl::PointCloud<InputPointType>::Ptr pTransformedXyzTargetCloud(
              new pcl::PointCloud<InputPointType>);
            pcl_ros::transformPointCloud(*pPointClouds_[t], *pTransformedXyzTargetCloud, transform);

            //--- initialize KD Search tree
            pcl::search::KdTree<InputPointType>::Ptr pKdTree(new pcl::search::KdTree<InputPointType>);
            pKdTree->setInputCloud(pTransformedXyzTargetCloud);

            //--- compute point normals Normal estimation ( if distance measure is set to point2surface)
            pcl::PointCloud<pcl::Normal>::Ptr pTargetNormals(new pcl::PointCloud<pcl::Normal>);
            if (distanceMeasure_ == POINT_2_SURFACE)
            {
                pcl::NormalEstimation<InputPointType, pcl::Normal> normalEstimation;
                normalEstimation.setInputCloud(pTransformedXyzTargetCloud);
                normalEstimation.setSearchMethod(pKdTree);
                normalEstimation.setKSearch(nNearestNeighbors_);
                normalEstimation.compute(*pTargetNormals);
            }

            //--- loop over source point cloud
            //--- for each point, find n nearest neighbors in transformed point cloud, calculate distance
            //--- of source point to this subset, create new point in pEnhancedSourceXyziCloud and store
            //--- normalized distance in intensity value

#pragma omp parallel shared(pKdTree, pTransformedXyzTargetCloud, \
                            pEnhancedSourceXyziCloud, pTargetNormals, calculateDistanceFn_)
            {
#pragma omp for
                for (pcl::PointCloud<pcl::PointXYZI>::iterator srcPntItr = pEnhancedSourceXyziCloud->begin();
                     srcPntItr != pEnhancedSourceXyziCloud->end();
                     ++srcPntItr)
                {
                    // XYZ-Poitn from source point cloud
                    InputPointType srcPntXyz;
                    srcPntXyz.x = srcPntItr->x;
                    srcPntXyz.y = srcPntItr->y;
                    srcPntXyz.z = srcPntItr->z;

                    // index of currently processed point in point cloud
                    int pntIdx = (srcPntItr - pEnhancedSourceXyziCloud->begin());

                    //--- find nearest neighbors
                    pcl::IndicesPtr pNearestNeighborsIndices(new pcl::Indices(nNearestNeighbors_));
                    std::vector<float> nearestNeighborsDistances(nNearestNeighbors_);
                    pKdTree->nearestKSearch(srcPntXyz, nNearestNeighbors_,
                                            *pNearestNeighborsIndices, nearestNeighborsDistances);

                    //--- calculate distance
                    float distance = calculateDistanceFn_(srcPntXyz, pTransformedXyzTargetCloud,
                                                          pTargetNormals,
                                                          pNearestNeighborsIndices,
                                                          nearestNeighborsDistances);

                    //--- truncate, normalize and store in intensity, if smaller than the wta-distance
                    if (distance < wtaDistance[pntIdx])
                    {
                        srcPntItr->intensity = (std::min(distance, maxDistance_) / maxDistance_);

                        wtaDistance[pntIdx] = distance;
                    }
                }
            }
        }

        //--- find points whose wta distance exceed clamp distance threshold and add to
        //--- pClampedPointIndices
        //--- see: https://stackoverflow.com/questions/12990148/get-all-positions-of-elements-in-stl-vector-that-are-greater-than-a-value
        auto distanceCmpFn = [&](float distance)
        {
            return distance > clampDistanceThreshold_;
        };
        auto itr = std::find_if(std::begin(wtaDistance), std::end(wtaDistance), distanceCmpFn);
        while (itr != std::end(wtaDistance))
        {
            pClampedPointIndices->emplace_back(std::distance(std::begin(wtaDistance), itr));
            itr = std::find_if(std::next(itr), std::end(wtaDistance), distanceCmpFn);
        }

        //--- remove point that have been marked by exceeding clampDistanceThreshold_ threshold
        pcl::ExtractIndices<pcl::PointXYZI> indexExtraction;
        indexExtraction.setIndices(pClampedPointIndices);
        indexExtraction.setNegative(true);
        indexExtraction.filterDirectly(pEnhancedSourceXyziCloud);

        //--- publish colorized point cloud
        if (!pEnhancedSourceXyziCloud->empty())
        {
            sensor_msgs::PointCloud2 cloudMsg;
            pcl::toROSMsg(*pEnhancedSourceXyziCloud, cloudMsg);
            cloudMsg.header = pointCloudMsgHeaders_[s];
            pubs_[s].publish(cloudMsg);
        }
    }

    //--- unlock mutex
    pointCloudMutex_.unlock();
}

//==================================================================================================
float PointCloud2PointCloudDistanceNodelet::calculatePoint2PointDistance(
  const InputPointType& iSourcePnt,
  const pcl::PointCloud<InputPointType>::Ptr& iTargetCloud,
  const pcl::PointCloud<pcl::Normal>::Ptr& iTargetNormals,
  const pcl::IndicesPtr& iNnIndices,
  const std::vector<float>& iNnDistances)
{
    UNUSED_VAR(iSourcePnt)
    UNUSED_VAR(iTargetCloud)
    UNUSED_VAR(iTargetNormals)
    UNUSED_VAR(iNnIndices)

    return iNnDistances[0];
}

//==================================================================================================
float PointCloud2PointCloudDistanceNodelet::calculatePoint2SurfaceDistance(
  const InputPointType& iSourcePnt,
  const pcl::PointCloud<InputPointType>::Ptr& iTargetCloud,
  const pcl::PointCloud<pcl::Normal>::Ptr& iTargetNormals,
  const pcl::IndicesPtr& iNnIndices,
  const std::vector<float>& iNnDistances)
{
    UNUSED_VAR(iNnDistances)

    //--- return value
    float wtaDistance = FLT_MAX;

    //--- loop over normal vectors and calculate distance by computing the dot product between
    //--- the query point and the normal vector
    //--- select the absolut smallest distance as return value
    for (pcl::Indices::iterator idxItr = iNnIndices->begin();
         idxItr != iNnIndices->end();
         ++idxItr)
    {
        Eigen::Vector3f srcPointVec(iSourcePnt.x, iSourcePnt.y, iSourcePnt.z);
        Eigen::Vector3f targetPointVec(iTargetCloud->at(*idxItr).x,
                                       iTargetCloud->at(*idxItr).y,
                                       iTargetCloud->at(*idxItr).z);
        Eigen::Vector3f targetNormalVec(iTargetNormals->at(*idxItr).normal_x,
                                        iTargetNormals->at(*idxItr).normal_y,
                                        iTargetNormals->at(*idxItr).normal_z);
        targetNormalVec.normalize();

        float d = -1 * targetPointVec.dot(targetNormalVec);

        float distance = std::abs(srcPointVec.dot(targetNormalVec) + d);
        if (distance < wtaDistance)
            wtaDistance = distance;
    }

    return wtaDistance;
}

} // namespace multisensor_calibration

// Export the class as a plugin using the PLUGINLIB_EXPORT_CLASS macro.
PLUGINLIB_EXPORT_CLASS(multisensor_calibration::PointCloud2PointCloudDistanceNodelet, nodelet::Nodelet)