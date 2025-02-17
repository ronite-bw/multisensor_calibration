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

#include "../include/multisensor_calibration/calibration/ExtrinsicLidarVehicleCalibrationNodelet.h"

// Std
#define USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <functional>
#include <future>
#include <thread>

// PCL
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

// ROS
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>

// small_gicp
#include <small_gicp/registration/registration_helper.hpp>

// multisensor_calibration
#include "../../include/multisensor_calibration/common/common.h"
#include "../../include/multisensor_calibration/common/utils.hpp"
#include "../../include/multisensor_calibration/data_processing/LocalPlaneSacModel.h"

namespace multisensor_calibration
{

//==================================================================================================
ExtrinsicLidarVehicleCalibrationNodelet::ExtrinsicLidarVehicleCalibrationNodelet() :
  Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>(EXTRINSIC_LIDAR_REFERENCE_CALIBRATION),
  sensorDataProcessingMutex_(dataProcessingMutex_),
  srcRegionMarkers_(),
  srcRegionSeedCloudPtrs_(),
  pSrcRegionsCloud_(nullptr),
  refRegionMarkers_(),
  refRegionSeedCloudPtrs_(),
  pRefRegionsCloud_(nullptr),
  regionMarkerHistory_(),
  pRefDataTransform_(nullptr)
{
    //--- set verbosity level for pcl
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    //--- connect dyn reconfigure callback
    dynConfigServer_.setCallback(
      boost::bind(&ExtrinsicLidarVehicleCalibrationNodelet::dynReconfigureCallback, this, _1, _2));

    //--- set reference name to 'vehicle'
    referenceName_ = "vehicle";
}

//==================================================================================================
ExtrinsicLidarVehicleCalibrationNodelet::~ExtrinsicLidarVehicleCalibrationNodelet()
{
}

//==================================================================================================
void ExtrinsicLidarVehicleCalibrationNodelet::computeRegionsCloud(
  const InputCloud_Message_T::ConstPtr& ipCloudMsg,
  const std::vector<InputPointType>& iRegionMarkers,
  std::vector<pcl::PointCloud<InputPointType>::Ptr>& oRegionSeedCloudPtrs,
  pcl::PointCloud<RegionPointType>::Ptr& opRegionsCloud) const
{
    // pointer to input point cloud from source sensor
    pcl::PointCloud<InputPointType>::Ptr pPointCloud(new pcl::PointCloud<InputPointType>);
    pcl::fromROSMsg(*ipCloudMsg, *pPointCloud);

    // pointer KD Search tree
    pcl::search::Search<InputPointType>::Ptr pSearchKdTree(new pcl::search::KdTree<InputPointType>);
    pSearchKdTree->setInputCloud(pPointCloud);

    //--- reset regions cloud
    opRegionsCloud.reset(new pcl::PointCloud<RegionPointType>);

    //--- resize list of seed clouds to size size of marker list
    while (oRegionSeedCloudPtrs.size() < iRegionMarkers.size())
        oRegionSeedCloudPtrs.push_back(nullptr);

    //--- loop over marker points, find nearest neighbor in cloud and do region growing
    for (std::vector<InputPointType>::const_iterator markerItr = iRegionMarkers.cbegin();
         markerItr != iRegionMarkers.cend();
         ++markerItr)
    {
        // Id of cluster
        int clusterIdx = std::distance(iRegionMarkers.cbegin(), markerItr);

        //--- reset cloud of region seed points
        oRegionSeedCloudPtrs.at(clusterIdx).reset(new pcl::PointCloud<InputPointType>);

        //--- find nearest neighbors
        std::vector<int> nnIndices(dynConfig_.num_nearest_neighbors);            // nearest neighbor indices
        std::vector<float> nnSquaredDistances(dynConfig_.num_nearest_neighbors); // nearest neighbor distances
        pSearchKdTree->nearestKSearch(*markerItr, dynConfig_.num_nearest_neighbors,
                                      nnIndices, nnSquaredDistances);

        // smart pointer to cluster indices
        pcl::PointIndices::Ptr pTmpPntIndices(new pcl::PointIndices());
        pTmpPntIndices->indices = nnIndices;

        //--- extract cluster indices
        pcl::ExtractIndices<InputPointType> extractIndices;
        extractIndices.setInputCloud(pPointCloud);
        extractIndices.setIndices(pTmpPntIndices);
        extractIndices.filter(*oRegionSeedCloudPtrs.at(clusterIdx));

        //--- detect plane around seed point with RANSAC
        //--- first only use the k nearest neighbors around seed point, then select select all
        //--- points that are within a distance threshold of the model.
        //--- this can further be restricted by using a confined local plane
        Eigen::VectorXf planeParameters;
        pcl::SampleConsensusModelPlane<InputPointType>::Ptr pPlaneSacModel = nullptr;

        //--- check which plane model to use
        if (dynConfig_.estimate_local_plane)
        {
            LocalPlaneSacModel<InputPointType>::Ptr pLocalPlaneSacModel(
              new LocalPlaneSacModel<InputPointType>(oRegionSeedCloudPtrs.at(clusterIdx)));
            Eigen::Vector3f seedPnt = Eigen::Vector3f(pPointCloud->at(nnIndices.front()).x,
                                                      pPointCloud->at(nnIndices.front()).y,
                                                      pPointCloud->at(nnIndices.front()).z);
            pLocalPlaneSacModel->setCenter(seedPnt);
            pLocalPlaneSacModel->setRadius(static_cast<float>(dynConfig_.local_plane_radius));
            pLocalPlaneSacModel->setIncrementalCenterUpdate(false);

            pPlaneSacModel = pLocalPlaneSacModel;
        }
        else
        {
            pPlaneSacModel.reset(
              new pcl::SampleConsensusModelPlane<InputPointType>(oRegionSeedCloudPtrs.at(clusterIdx)));
        }

        //--- compute model by Ransac
        pcl::RandomSampleConsensus<InputPointType> planeRansac(pPlaneSacModel);
        planeRansac.setDistanceThreshold(dynConfig_.distance_thresh);
        bool isSuccessful = planeRansac.computeModel();
        if (!isSuccessful)
            continue;
        planeRansac.refineModel();
        planeRansac.getModelCoefficients(planeParameters);

        //--- select cluster from full input cloud by using estimated plane coefficients
        pcl::PointCloud<InputPointType>::Ptr pTmpClusterCloud(new pcl::PointCloud<InputPointType>);
        pTmpPntIndices.reset(new pcl::PointIndices());

        pPlaneSacModel->setInputCloud(pPointCloud);
        pPlaneSacModel->selectWithinDistance(planeParameters, dynConfig_.distance_thresh, pTmpPntIndices->indices);

        extractIndices.setIndices(pTmpPntIndices);
        extractIndices.filter(*pTmpClusterCloud);

        //--- set index of region marker to intensity, normal vector of sac model and push to output
        std::for_each(pTmpClusterCloud->begin(), pTmpClusterCloud->end(),
                      [&](InputPointType& iPnt)
                      {
                          RegionPointType oPnt;
                          oPnt.x         = iPnt.x;
                          oPnt.y         = iPnt.y;
                          oPnt.z         = iPnt.z;
                          oPnt.intensity = static_cast<float>(clusterIdx);
#ifdef USE_NORMAL_INFO
                          oPnt.normal_x = planeParameters[0];
                          oPnt.normal_y = planeParameters[1];
                          oPnt.normal_z = planeParameters[2];
#endif

                          //--- added point to cumulative cloud
                          opRegionsCloud->push_back(oPnt);
                      });
    }
}

//==================================================================================================
void ExtrinsicLidarVehicleCalibrationNodelet::doCoarseCalibration()
{
    using namespace pcl::registration;

    // check that enough seed data is available
    if (srcRegionSeedCloudPtrs_.size() < calibrationItrCnt_ ||
        refRegionSeedCloudPtrs_.size() < calibrationItrCnt_)
        return;

    //--- merge seed clouds used for single calibration iteration in order to compute a
    //--- good guess for the gicp

    // Pointer to seed cloud of source region
    pcl::PointCloud<InputPointType>::Ptr pSrcRegionSeedCloud(new pcl::PointCloud<InputPointType>);
    // Pointer to seed cloud of reference region
    pcl::PointCloud<InputPointType>::Ptr pRefRegionSeedCloud(new pcl::PointCloud<InputPointType>);
    for (uint i = 0; i < calibrationItrCnt_; ++i)
    {
        pSrcRegionSeedCloud->insert(pSrcRegionSeedCloud->end(),
                                    srcRegionSeedCloudPtrs_[i]->begin(),
                                    srcRegionSeedCloudPtrs_[i]->end());
        pRefRegionSeedCloud->insert(pRefRegionSeedCloud->end(),
                                    refRegionSeedCloudPtrs_[i]->begin(),
                                    refRegionSeedCloudPtrs_[i]->end());
    }

    // Correspondences of marker corners between src and ref cloud. Since the marker corners are
    // extracted in order, the correspondence list simply consists of increasing index numbers
    pcl::Correspondences correspondences;
    for (uint i = 0; i < pSrcRegionSeedCloud->size(); ++i)
    {
        correspondences.push_back(pcl::Correspondence(i, i, 1.f));
    }

    //--- estimate sensor extrinsics from marker corners
    auto sensorExtrinsic = computeExtrinsicsFromPointCorrespondences<InputPointType>(
      pSrcRegionSeedCloud, pRefRegionSeedCloud, correspondences);
    sensorExtrinsics_.push_back(sensorExtrinsic);

    //--- increment iteration count
    calibrationItrCnt_++;

    // message publishing calibration result
    CalibrationResultMsg calibResultMsg;
    calibResultMsg.isSuccessful = true;
    calibResultPub_.publish(calibResultMsg);
}

//==================================================================================================
void ExtrinsicLidarVehicleCalibrationNodelet::dynReconfigureCallback(
  ExtrinsicLidarVehicleCalibrationConfig& iConfig, uint32_t level)
{
    UNUSED_VAR(level);

    //--- store values
    dynConfig_ = iConfig;
}

//==================================================================================================
void ExtrinsicLidarVehicleCalibrationNodelet::finalizeCalibration()
{
    //--- do statistical outlier filtering
    pcl::StatisticalOutlierRemoval<RegionPointType> sorFilter;
    sorFilter.setMeanK(50);            // TODO: dynconfig
    sorFilter.setStddevMulThresh(1.0); // TODO: dynconfig
    sorFilter.setInputCloud(pSrcRegionsCloud_);
    sorFilter.filter(*pSrcRegionsCloud_);
    sorFilter.setInputCloud(pRefRegionsCloud_);
    sorFilter.filter(*pRefRegionsCloud_);

    if (pSrcRegionsCloud_->empty() || pRefRegionsCloud_->empty())
    {
        ROS_ERROR("[%s] Could not finalize calibration. No common observations available.",
                  nodeletName_.c_str());
        return;
    }

    //--- run GICP
    double icpRmse = runIcp<RegionPointType>(
      pSrcRegionsCloud_, pRefRegionsCloud_,
      static_cast<small_gicp::RegistrationSetting::RegistrationType>(dynConfig_.registration_icp_variant),
      dynConfig_.registration_icp_max_correspondence_distance,
      dynConfig_.registration_icp_rotation_tolerance,
      dynConfig_.registration_icp_translation_tolerance);
    calibResult_.error = std::make_pair("Root Mean Squared Error (in m)", icpRmse);

    //--- set calibration meta data
    calibResult_.calibrations[0].srcSensorName = srcSensorName_;
    calibResult_.calibrations[0].srcFrameId    = srcFrameId_;
    calibResult_.calibrations[0].refSensorName = refSensorName_;
    calibResult_.calibrations[0].refFrameId    = refFrameId_;
    calibResult_.calibrations[0].baseFrameId   = baseFrameId_;

    //--- get extrinsics
    const lib3d::Extrinsics& extrinsics = sensorExtrinsics_.back();

    //--- get transformation from lib3d::Extrinsics.
    // resulting transformation from ref to src sensor
    tf::Transform refToSrcTransform;
    utils::setTfTransformFromCameraExtrinsics(extrinsics,
                                              refToSrcTransform);
    calibResult_.calibrations[0].XYZ = refToSrcTransform.inverse().getOrigin(); // invert to get LOCAL_2_REF
    double roll, pitch, yaw;
    refToSrcTransform.inverse().getBasis().getRPY(roll, pitch, yaw); // invert to get LOCAL_2_REF
    calibResult_.calibrations[0].RPY = tf::Vector3(roll, pitch, yaw);

    //--- store meta information into calibResult
    calibResult_.numObservations = static_cast<int>(refRegionMarkers_.size());

    //--- print out final transformation
    ROS_INFO("[%s]"
             "\n==================================================================================="
             "\n%s"
             "\n===================================================================================",
             nodeletName_.c_str(), calibResult_.toString().c_str());

    // message publishing calibration result
    CalibrationResultMsg calibResultMsg;
    calibResultMsg.isSuccessful = true;
    calibResultPub_.publish(calibResultMsg);
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibrationNodelet::initializePublishers(ros::NodeHandle& ioNh)
{
    bool isSuccessful = CalibrationBase::initializePublishers(ioNh);

    srcRegionPub_ = ioNh.advertise<RoisCloud_Message_T>(
      srcSensorName_ + "/" + ROIS_CLOUD_TOPIC_NAME, 10);

    refRegionPub_ = ioNh.advertise<RoisCloud_Message_T>(
      referenceName_ + "/" + ROIS_CLOUD_TOPIC_NAME, 10);

    return isSuccessful;
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibrationNodelet::initializeServices(ros::NodeHandle& ioNh)
{
    bool isSuccessful = ExtrinsicCalibrationBase::initializeServices(ioNh);

    //--- shutdown service to capture calibration target, since not needed
    CalibrationBase::captureSrv_.shutdown();

    //--- add region marker service
    regionMarkerSrv_ = ioNh.advertiseService(
      ADD_REGION_MARKER_SRV_NAME,
      &ExtrinsicLidarVehicleCalibrationNodelet::onRequestAddRegionMarker, this);

    return isSuccessful;
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibrationNodelet::initializeSubscribers(ros::NodeHandle& ioNh)
{
    //--- subscribe to topics with clicked point from rviz
    clickedPointSubsc_ = ioNh.subscribe<geometry_msgs::PointStamped>(
      "/clicked_point", 1,
      boost::bind(&ExtrinsicLidarVehicleCalibrationNodelet::onPointClicked, this, _1));

    //--- subscribe to topics with name cloudTopic
    srcCloudSubsc_ = ioNh.subscribe<InputCloud_Message_T>(
      srcLidarCloudTopic_, 1,
      boost::bind(&ExtrinsicLidarVehicleCalibrationNodelet::onSensorDataReceived, this, _1));
    refCloudSubsc_ = ioNh.subscribe<InputCloud_Message_T>(
      refLidarCloudTopic_, 1,
      boost::bind(&ExtrinsicLidarVehicleCalibrationNodelet::onReferenceDataReceived, this, _1));

    return true;
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibrationNodelet::initializeWorkspaceObjects()
{
    bool retVal = true;

    retVal &= CalibrationBase::initializeWorkspaceObjects();

    //--- initialize calibration workspace
    fs::path calibWsPath = robotWsPath_;
    calibWsPath /=
      std::string(srcLidarSensorName_ + "_vehicle_extrinsic_calibration");
    pCalibrationWs_ =
      std::make_shared<ExtrinsicLidarVehicleCalibWorkspace>(calibWsPath, nodeletName_);
    retVal &= (pCalibrationWs_ != nullptr);

    //--- copy calibration target config to workspace
    if (pCalibrationWs_)
    {
        //--- create backup
        retVal &= pCalibrationWs_->createBackup();
    }

    return retVal;
}

//==================================================================================================
void ExtrinsicLidarVehicleCalibrationNodelet::onInit()
{
    //--- get global and private node handle
    nh_  = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    //--- set nodelet name
    nodeletName_ = Nodelet::getName();

    //--- read launch parameters
    isInitialized_ = readLaunchParameters(pnh_);

    //--- if reading of launch parameters has returned with false, i.e. if error occurred, return.
    if (isInitialized_ == false)
        return;

    //--- initialize services
    isInitialized_ &= initializeServices(pnh_);

    //--- initialize workspace objects
    isInitialized_ &= initializeWorkspaceObjects();

    //--- create and start calibration workflow;
    isInitialized_ &= createAndStartCalibrationWorkflow(nh_);
}

//==================================================================================================
void ExtrinsicLidarVehicleCalibrationNodelet::onPointClicked(
  const geometry_msgs::PointStampedConstPtr& ipPointMsg)
{
    //--- create service message and call method
    multisensor_calibration::AddRegionMarker srvMsg;
    srvMsg.request.sensorFrameId = ipPointMsg->header.frame_id;
    srvMsg.request.point         = ipPointMsg->point;
    if (onRequestAddRegionMarker(srvMsg.request, srvMsg.response))
    {
        ROS_INFO("[%s] %s", nodeletName_.c_str(), srvMsg.response.msg.c_str());
    }
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibrationNodelet::onRequestAddRegionMarker(
  multisensor_calibration::AddRegionMarker::Request& iReq,
  multisensor_calibration::AddRegionMarker::Response& oRes)
{
    // Marker point to store in list
    InputPointType pnt;
    pnt.x = static_cast<float>(iReq.point.x);
    pnt.y = static_cast<float>(iReq.point.y);
    pnt.z = static_cast<float>(iReq.point.z);

    if (iReq.sensorFrameId == refFrameId_)
    {
        //--- get ownership of mutex
        std::lock_guard<std::mutex> guard(refDataProcessingMutex_);

        refRegionMarkers_.push_back(pnt);
        regionMarkerHistory_.push_back(refFrameId_);

        oRes.isAccepted = true;
        oRes.msg        = "Added marker position to reference.";
    }
    else if (iReq.sensorFrameId == srcFrameId_)
    {
        //--- get ownership of mutex
        std::lock_guard<std::mutex> guard(sensorDataProcessingMutex_);

        srcRegionMarkers_.push_back(pnt);
        regionMarkerHistory_.push_back(srcFrameId_);

        oRes.isAccepted = true;
        oRes.msg        = "Added marker position to source.";
    }
    else
    {
        oRes.isAccepted = false;
        oRes.msg        = "No valid sensor frame ID.";
    }

    return true;
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibrationNodelet::onRequestRemoveObservation(
  multisensor_calibration::RemoveLastObservation::Request& iReq,
  multisensor_calibration::RemoveLastObservation::Response& oRes)
{
    UNUSED_VAR(iReq);

    //--- get ownership of mutex
    std::lock_guard<std::mutex> srcGuard(sensorDataProcessingMutex_);
    std::lock_guard<std::mutex> refGuard(refDataProcessingMutex_);

    //--- check in history list for last frame ID and remove from appropriate list.
    if (!regionMarkerHistory_.empty())
    {
        oRes.isAccepted = true;

        if (regionMarkerHistory_.back() == refFrameId_)
        {
            refRegionMarkers_.pop_back();
            oRes.msg = "Removed last marker from reference.";
        }
        else if (regionMarkerHistory_.back() == srcFrameId_)
        {
            srcCloudFrameId_.pop_back();
            oRes.msg = "Removed last marker from source.";
        }
        else
        {
            oRes.isAccepted = false;
            oRes.msg        = "Could not find frame ID in history list.";
        }

        regionMarkerHistory_.pop_back();
        if (calibrationItrCnt_ > 1)
        {
            calibrationItrCnt_--;
            sensorExtrinsics_.pop_back();
        }
    }
    else
    {
        oRes.isAccepted = false;
        oRes.msg        = "No region marker in history list.";
    }

    ROS_INFO("[%s] %s", nodeletName_.c_str(), oRes.msg.c_str());

    return true;
}

//==================================================================================================
void ExtrinsicLidarVehicleCalibrationNodelet::onReferenceDataReceived(
  const InputCloud_Message_T::ConstPtr& ipRefCloudMsg)
{
    //--- check if nodelet is initialized
    if (!isInitialized_)
    {
        ROS_ERROR("[%s]"
                  "\n\t> Nodelet is not initialized.",
                  nodeletName_.c_str());
        return;
    }

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(refDataProcessingMutex_);

    //--- set frame ids and initialize sensor extrinsics if applicable
    if (refFrameId_ != ipRefCloudMsg->header.frame_id)
    {
        refFrameId_ = ipRefCloudMsg->header.frame_id;

        //--- if base frame id is not empty and unequal to refCloudFrameId also get transform
        //--- between refFrameID and baseFrameID and pass to refLidarProcessor.
        if (!baseFrameId_.empty() && baseFrameId_ != refFrameId_)
        {
            if (tfListener_.frameExists(baseFrameId_))
            {
                try
                {
                    tf::StampedTransform transform;
                    tfListener_.lookupTransform(baseFrameId_, refFrameId_,
                                                ros::Time(0), transform);
                    pRefDataTransform_ = std::make_shared<tf::Transform>(transform);
                }
                catch (tf::TransformException& ex)
                {
                    ROS_ERROR("[%s]"
                              "\n\t> tf::TransformException: %s",
                              nodeletName_.c_str(), ex.what());
                }
            }
            else
            {
                ROS_WARN("[%s]"
                         "\n\t> Base Frame %s does not exists! "
                         "Removing base frame and calibrating relative to reference cloud.",
                         nodeletName_.c_str(), baseFrameId_.c_str());
                baseFrameId_       = "";
                pRefDataTransform_ = nullptr;
            }
        }
    }

    //--- return immediately if no markers are set
    if (refRegionMarkers_.empty())
        return;

    //--- compute regions for calibration
    computeRegionsCloud(ipRefCloudMsg, refRegionMarkers_, refRegionSeedCloudPtrs_, pRefRegionsCloud_);

    //--- transform reference data if applicable
    if (pRefDataTransform_)
    {
        pcl::PointCloud<InputPointType>::Ptr pTmpCloud(new pcl::PointCloud<InputPointType>);
        pcl_ros::transformPointCloud(*pRefRegionsCloud_, *pTmpCloud, *pRefDataTransform_);

        //--- swap pointer between input and temporary cloud
        pTmpCloud.swap(pRefRegionsCloud_);
        pTmpCloud->clear();
    }

    //--- publish clustered regions
    if (pRefRegionsCloud_)
    {
        RoisCloud_Message_T cloudMsg;
        pcl::toROSMsg(*pRefRegionsCloud_, cloudMsg);
        cloudMsg.header.stamp    = ipRefCloudMsg->header.stamp;
        cloudMsg.header.frame_id = (pRefDataTransform_) ? baseFrameId_ : refFrameId_;
        refRegionPub_.publish(cloudMsg);
    }

    //--- If calibration iteration count is equal to the smallest number of region markers, i.e.
    //--- if a new region marker has been added and not yet been calibrated, run single calibration
    //--- iteration. In this the calibration iteration count will also be increased by one.
    if (std::min(srcRegionMarkers_.size(), refRegionMarkers_.size()) == calibrationItrCnt_)
        doCoarseCalibration();
}

//==================================================================================================
void ExtrinsicLidarVehicleCalibrationNodelet::onSensorDataReceived(
  const InputCloud_Message_T::ConstPtr& ipSrcCloudMsg)
{
    //--- check if nodelet is initialized
    if (!isInitialized_)
    {
        ROS_ERROR("[%s]"
                  "\n\t> Nodelet is not initialized.",
                  nodeletName_.c_str());
        return;
    }

    //--- get ownership of mutex
    std::lock_guard<std::mutex> guard(sensorDataProcessingMutex_);

    //--- set frame ids and initialize sensor extrinsics if applicable
    if (srcFrameId_ != ipSrcCloudMsg->header.frame_id)
    {
        srcFrameId_ = ipSrcCloudMsg->header.frame_id;
    }

    //--- return immediately if no markers are set
    if (srcRegionMarkers_.empty())
        return;

    //--- compute regions for calibration
    computeRegionsCloud(ipSrcCloudMsg, srcRegionMarkers_, srcRegionSeedCloudPtrs_, pSrcRegionsCloud_);

    //--- publish clustered regions
    if (pSrcRegionsCloud_)
    {
        RoisCloud_Message_T cloudMsg;
        pcl::toROSMsg(*pSrcRegionsCloud_, cloudMsg);
        cloudMsg.header = ipSrcCloudMsg->header;
        srcRegionPub_.publish(cloudMsg);
    }
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibrationNodelet::saveCalibrationSettingsToWorkspace()
{
    if (!ExtrinsicCalibrationBase::saveCalibrationSettingsToWorkspace())
        return false;

    QSettings* pCalibSettings = pCalibrationWs_->settingsPtr();
    if (!pCalibSettings)
        return false;

    //--- source lidar sensor name
    pCalibSettings->setValue("source_lidar/sensor_name",
                             QString::fromStdString(srcLidarSensorName_));

    //--- source lidar cloud topic
    pCalibSettings->setValue("source_lidar/cloud_topic",
                             QString::fromStdString(srcLidarCloudTopic_));

    //--- reference cloud topic
    pCalibSettings->setValue("reference/cloud_topic",
                             QString::fromStdString(refLidarCloudTopic_));

    //--- sync settings file
    pCalibSettings->sync();

    return true;
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibrationNodelet::readLaunchParameters(const ros::NodeHandle& iNh)
{
    if (!ExtrinsicCalibrationBase::readLaunchParameters(iNh))
        return false;

    //--- source_lidar_sensor_name
    srcLidarSensorName_ =
      readStringLaunchParameter(iNh, "src_lidar_sensor_name", DEFAULT_LIDAR_SENSOR_NAME);

    //--- source_lidar_cloud_topic
    srcLidarCloudTopic_ =
      readStringLaunchParameter(iNh, "src_lidar_cloud_topic", DEFAULT_LIDAR_CLOUD_TOPIC);

    //--- reference_lidar_cloud_topic
    refLidarCloudTopic_ =
      readStringLaunchParameter(iNh, "ref_lidar_cloud_topic", DEFAULT_LIDAR_CLOUD_TOPIC);

    return true;
}

//==================================================================================================
void ExtrinsicLidarVehicleCalibrationNodelet::reset()
{
    ExtrinsicCalibrationBase::reset();

    srcRegionMarkers_.clear();
    srcRegionSeedCloudPtrs_.clear();
    pSrcRegionsCloud_->clear();
    refRegionMarkers_.clear();
    refRegionSeedCloudPtrs_.clear();
    pRefRegionsCloud_->clear();

    regionMarkerHistory_.clear();
}

//==================================================================================================
bool ExtrinsicLidarVehicleCalibrationNodelet::shutdownSubscribers()
{
    //--- check if node is initialized
    if (!isInitialized_)
        return false;

    //--- get ownership of mutex
    std::lock_guard<std::mutex> srcGuard(sensorDataProcessingMutex_);
    std::lock_guard<std::mutex> refGuard(refDataProcessingMutex_);

    //--- unsubscribe subscribers
    srcCloudSubsc_.shutdown();
    refCloudSubsc_.shutdown();

    return true;
}

} // namespace multisensor_calibration

// Export the class as a plugin using the PLUGINLIB_EXPORT_CLASS macro.
PLUGINLIB_EXPORT_CLASS(multisensor_calibration::ExtrinsicLidarVehicleCalibrationNodelet, nodelet::Nodelet)