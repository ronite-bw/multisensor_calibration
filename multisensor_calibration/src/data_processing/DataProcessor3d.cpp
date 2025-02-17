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

#include "multisensor_calibration/data_processing/DataProcessor3d.h"

// Std
#include <iostream>
#include <regex>

// ROS
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/transforms.h>

// multisensor_calibration
#include "../include/multisensor_calibration/common/utils.hpp"
#include <multisensor_calibration/LidarDataProcessingNodeletConfig.h>

namespace multisensor_calibration
{

//==================================================================================================
DataProcessor3d::DataProcessor3d(const std::string& iNodeletName,
                                 const std::string& iSensorName,
                                 const fs::path& iCalibTargetFilePath) :
  SensorDataProcessorBase<pcl::PointCloud<InputPointType>>(iNodeletName,
                                                           iSensorName,
                                                           iCalibTargetFilePath),
  pDataTransform_(nullptr),
  pRoisCloud_(nullptr),
  inputCloudPtrs_(),
  calibrationTargetCloudPtrs_(),
  estimatedMarkerCornersCloudPtrs_()
{
}

//==================================================================================================
DataProcessor3d::~DataProcessor3d()
{
}

//==================================================================================================
void DataProcessor3d::setDataTransform(const std::shared_ptr<tf::Transform>& pTransform)
{
    pDataTransform_ = pTransform;
}

//==================================================================================================
std::vector<pcl::PointCloud<InputPointType>::Ptr>
DataProcessor3d::getCalibrationTargetCloudPtrs() const
{
    return calibrationTargetCloudPtrs_;
}

//==================================================================================================
pcl::PointCloud<InputPointType>::Ptr DataProcessor3d::getLastCalibrationTargetCloud() const
{
    return calibrationTargetCloudPtrs_.back();
}

//==================================================================================================

pcl::PointCloud<InputPointType>::Ptr DataProcessor3d::getLastInputCloud() const
{
    return inputCloudPtrs_.back();
}

//==================================================================================================
std::vector<std::array<cv::Point3f, 4>> DataProcessor3d::getLastEstimatedMarkerCorners() const
{
    return estimatedMarkerCorners_.back();
}

//==================================================================================================
std::vector<uint> DataProcessor3d::getLastEstimatedMarkerIds() const
{
    return estimatedMarkerIds_.back();
}

//==================================================================================================
pcl::PointCloud<pcl::PointXYZI>::Ptr DataProcessor3d::getLastMarkerCornersCloud() const
{
    return estimatedMarkerCornersCloudPtrs_.back();
}

//==================================================================================================
void DataProcessor3d::getOrderedObservations(std::set<uint>& oObservationIds,
                                             std::vector<cv::Point3f>& oCornerObservations,
                                             const int& iIterationBegin,
                                             const int& iNumIterations) const
{
    oObservationIds.clear();
    oCornerObservations.clear();

    //--- if iteration begin is larger than the amount of iterations available, return
    if (iIterationBegin > static_cast<int>(estimatedMarkerIds_.size()))
        return;

    //--- set begin iterator
    auto estimatedMarkerIdsItr = estimatedMarkerIds_.cbegin() + (iIterationBegin - 1);

    // number of iterations
    uint tmpNumIterations = (iNumIterations <= 0) ? estimatedMarkerIds_.size() : iNumIterations;

    //--- loop through iterations
    for (uint i = 0;
         i < tmpNumIterations && estimatedMarkerIdsItr != estimatedMarkerIds_.end();
         ++i, ++estimatedMarkerIdsItr)
    {
        int calibrationItrIdx = std::distance(estimatedMarkerIds_.cbegin(), estimatedMarkerIdsItr);

        //--- loop through all markers in this iteration
        for (uint m = 0; m < estimatedMarkerIds_[calibrationItrIdx].size(); ++m)
        {
            int markerId = estimatedMarkerIds_[calibrationItrIdx][m];

            //--- construct observationId, insert into list and get position
            uint obsId = (calibrationItrIdx + 1) * 100 + markerId; // observation id
            std::pair<std::set<uint>::iterator, bool> retVal =
              oObservationIds.insert(obsId);

            // index of marker observation in list
            int obsIdx = std::distance(oObservationIds.begin(), retVal.first);

            //--- loop over corners and insert into list
            for (uchar c = 0; c < 4; ++c)
            {
                cv::Point3f markerCorner = estimatedMarkerCorners_[calibrationItrIdx][m][c];

                // insertion position
                auto insertionPos = oCornerObservations.begin() + obsIdx * 4 + c;
                oCornerObservations.insert(insertionPos, markerCorner);
            }
        }
    }
}

//==================================================================================================
pcl::PointCloud<InputPointType>::Ptr DataProcessor3d::getRegionOfInterestsCloudPtr() const
{
    return pRoisCloud_;
}

//==================================================================================================
bool DataProcessor3d::getSensorDataFromMsg(const InputCloud_Message_T::ConstPtr& ipMsg,
                                           pcl::PointCloud<InputPointType>& oData) const
{
    pcl::fromROSMsg(*ipMsg, oData);

    return true;
}

//==================================================================================================
bool DataProcessor3d::initializePublishers(ros::NodeHandle& ioNh)
{
    // lambda to append sensorName_ (if not empty) to topic name
    auto constructTopicName = [&](const std::string& iTopicName) -> std::string
    {
        return (sensorName_.empty()) ? iTopicName : sensorName_ + "/" + iTopicName;
    };

    roisCloudPub_ = ioNh.advertise<RoisCloud_Message_T>(
      constructTopicName(ROIS_CLOUD_TOPIC_NAME), 10);

    markerCornersPub_ = ioNh.advertise<MarkerCornerCloud_Message_T>(
      constructTopicName(MARKER_CORNERS_TOPIC_NAME), 10);

    targetCloudPub_ = ioNh.advertise<TargetPatternCloud_Message_T>(
      constructTopicName(TARGET_PATTERN_CLOUD_TOPIC_NAME), 10);

    targetBoardPosePub_ = ioNh.advertise<TargetBoardPose_Message_T>(
      constructTopicName(TARGET_BOARD_POSE_TOPIC_NAME), 10);

    return true;
}

//==================================================================================================
bool DataProcessor3d::initializeServices(ros::NodeHandle& ioNh)
{
    addMarkerObsSrv_ = ioNh.advertiseService(
      sensorName_ + "/" + ADD_MARKER_OBS_SRV_NAME,
      &DataProcessor3d::onAddMarkerObservations, this);

    importMarkerObsSrv_ = ioNh.advertiseService(
      sensorName_ + "/" + IMPORT_MARKER_OBS_SRV_NAME,
      &DataProcessor3d::onImportMarkerObservations, this);

    return true;
}

//==================================================================================================
void DataProcessor3d::publishPreview(const std_msgs::Header& iHeader) const
{
    //--- publish regions of interest
    if (pRoisCloud_)
    {
        RoisCloud_Message_T cloudMsg;
        pcl::toROSMsg(*pRoisCloud_, cloudMsg);
        cloudMsg.header = iHeader;
        roisCloudPub_.publish(cloudMsg);
    }
}

//==================================================================================================
void DataProcessor3d::publishLastTargetDetection(const std_msgs::Header& iHeader) const
{
    //--- publish target point cloud
    auto pTargetCloud = calibrationTargetCloudPtrs_.back();
    if (pTargetCloud)
    {
        TargetPatternCloud_Message_T cloudMsg;
        pcl::toROSMsg(*pTargetCloud, cloudMsg);
        cloudMsg.header = iHeader;
        targetCloudPub_.publish(cloudMsg);

        ROS_INFO("[%s] Published target cloud!", getLoggingId().c_str());
    }

    //--- publish marker corners point cloud
    auto pMarkerCloud = estimatedMarkerCornersCloudPtrs_.back();
    if (pMarkerCloud)
    {
        MarkerCornerCloud_Message_T cloudMsg;
        pcl::toROSMsg(*pMarkerCloud, cloudMsg);
        cloudMsg.header = iHeader;
        markerCornersPub_.publish(cloudMsg);

        ROS_INFO("[%s] Published 3D points of marker corners!",
                 getLoggingId().c_str());
    }

    //--- Publish multi-array with up vector and center of detected target
    if (!capturedCalibTargetPoses_.empty())
        publishLastCalibrationTargetPose(iHeader, capturedCalibTargetPoses_.back(), true, targetBoardPosePub_);
}

//==================================================================================================
bool DataProcessor3d::removeCalibIteration(const uint& iIterationId)
{
    bool isSuccessful = SensorDataProcessorBase::removeCalibIteration(iIterationId);

    if (!isSuccessful)
        return false;

    inputCloudPtrs_.erase(inputCloudPtrs_.begin() +
                          (iIterationId - 1));
    calibrationTargetCloudPtrs_.erase(calibrationTargetCloudPtrs_.begin() +
                                      (iIterationId - 1));
    estimatedMarkerIds_.erase(estimatedMarkerIds_.begin() +
                              (iIterationId - 1));
    estimatedMarkerCorners_.erase(estimatedMarkerCorners_.begin() +
                                  (iIterationId - 1));
    estimatedMarkerCornersCloudPtrs_.erase(estimatedMarkerCornersCloudPtrs_.begin() +
                                           (iIterationId - 1));

    return true;
}

//==================================================================================================
void DataProcessor3d::reset()
{
    SensorDataProcessorBase::reset();

    if (pRoisCloud_)
        pRoisCloud_ = nullptr;
    inputCloudPtrs_.clear();
    calibrationTargetCloudPtrs_.clear();
    estimatedMarkerIds_.clear();
    estimatedMarkerCorners_.clear();
    estimatedMarkerCornersCloudPtrs_.clear();
}

//==================================================================================================
bool DataProcessor3d::saveObservations(fs::path iOutputPath) const
{

    bool isSuccessful = SensorDataProcessorBase::saveObservations(iOutputPath);
    if (!isSuccessful)
        return false;

    //--- loop through calibration iterations
    for (uint calibItr = 0; calibItr < capturedCalibTargetPoses_.size(); ++calibItr)
    {
        try
        {
            // directory of each calibration iteration
            fs::path iterationOutputPath = iOutputPath;
            iterationOutputPath.append(std::to_string(calibItr + 1));
            if (!fs::exists(iterationOutputPath))
                fs::create_directories(iterationOutputPath);

            //--- target detection in cloud
            pcl::PointCloud<InputPointType>::Ptr mergedCloud(new pcl::PointCloud<InputPointType>());
            *mergedCloud += *inputCloudPtrs_[calibItr];
            *mergedCloud += *calibrationTargetCloudPtrs_[calibItr];
            *mergedCloud += *estimatedMarkerCornersCloudPtrs_[calibItr];
            pcl::io::savePLYFileBinary(
              iterationOutputPath.string() + "/" + sensorName_ + ANNOTATED_CLOUD_FILE_SUFFIX,
              *mergedCloud);

            //--- marker corner observations
            writeMarkerObservationsToFile(
              iterationOutputPath.string() + "/" + sensorName_ + OBSERVATIONS_FILE_SUFFIX,
              estimatedMarkerIds_[calibItr], estimatedMarkerCorners_[calibItr]);
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("[%s] %s", nodeletName_.c_str(), e.what());
            isSuccessful = false;
        }
    }

    return isSuccessful;
}

//==================================================================================================
bool DataProcessor3d::onAddMarkerObservations(
  multisensor_calibration::AddMarkerObservations::Request& iReq,
  multisensor_calibration::AddMarkerObservations::Response& oRes)
{
    if (!isInitialized_)
    {
        oRes.isAccepted = false;
        oRes.msg        = "Data processor not initialized.";

        ROS_ERROR("[%s] %s", getLoggingId().c_str(), oRes.msg.c_str());

        return false;
    }

    if (iReq.observation.marker_ids.size() != 4 ||
        iReq.observation.marker_top_left_point.size() != 4)
    {
        oRes.isAccepted = false;
        oRes.msg        = "For each observation there need to be four marker IDs and four marker "
                          "points. This could be changed in future.";

        ROS_ERROR("[%s] %s", getLoggingId().c_str(), oRes.msg.c_str());

        return false;
    }

    //--- transform marker corners into base frame if transform is available,
    //--- first a conversion from the request to a point cloud is done
    //--- then, the point cloud is transformed

    // pointer to point cloud container holding top left marker corners
    pcl::PointCloud<pcl::PointXYZ>::Ptr pMarkerTlPoints(new pcl::PointCloud<pcl::PointXYZ>);

    // pointer to temporary point cloud container
    pcl::PointCloud<pcl::PointXYZ>::Ptr pTmpCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (auto point : iReq.observation.marker_top_left_point)
        pMarkerTlPoints->push_back(pcl::PointXYZ(
          static_cast<float>(point.x),
          static_cast<float>(point.y),
          static_cast<float>(point.z)));

    if (pDataTransform_)
    {
        pcl_ros::transformPointCloud(*pMarkerTlPoints, *pTmpCloud, *pDataTransform_);

        //--- swap pointer between input and temporary cloud
        pTmpCloud.swap(pMarkerTlPoints);
        pTmpCloud->clear();
    }

    std::vector<cv::Point3f> vMarkerTlPoints;
    for (const auto& point : *pMarkerTlPoints)
        vMarkerTlPoints.push_back(cv::Point3f(point.x, point.y, point.z));

    //--- compute pose from marker corners
    //--- Currently it assumed that there are 4 marker and 4 corner points.
    //--- TODO: This should be made more flexible in future.
    Eigen::Vector3f center, up, right, normal;
    lib3d::Extrinsics targetPose;
    calibrationTarget_.computePoseFromTopLeftMarkerCorners(
      iReq.observation.marker_ids, vMarkerTlPoints, center, up, right, normal);
    calibrationTarget_.computePose(center, up, normal, targetPose);

    Eigen::Matrix4f poseTransform(Eigen::Matrix4f::Identity());
    poseTransform.block(0, 0, 3, 1) = right;
    poseTransform.block(0, 1, 3, 1) = up;
    poseTransform.block(0, 2, 3, 1) = normal;
    poseTransform.block(0, 3, 3, 1) = center;

    //--- store empty input cloud
    inputCloudPtrs_.push_back(nullptr);
    inputCloudPtrs_.back().reset(new pcl::PointCloud<InputPointType>());

    //--- store calibration target cloud and Rois Cloud
    //--- get reference cloud and transform according to pose
    pcl::PointCloud<InputPointType>::Ptr pTargetCloud(new pcl::PointCloud<InputPointType>);
    pcl::io::loadPLYFile(calibrationTarget_.cadModelCloudPath, *pTargetCloud);

    pRoisCloud_.reset(new pcl::PointCloud<InputPointType>());
    calibrationTargetCloudPtrs_.push_back(nullptr);
    calibrationTargetCloudPtrs_.back().reset(new pcl::PointCloud<InputPointType>());
    pcl::transformPointCloud(*pTargetCloud, *pRoisCloud_, poseTransform);
    pcl::transformPointCloud(*pTargetCloud, *calibrationTargetCloudPtrs_.back(), poseTransform);

    //--- store marker ids
    estimatedMarkerIds_.push_back(iReq.observation.marker_ids);

    //--- store marker corners
    std::vector<std::array<cv::Point3f, 4>> markerCorners;
    calibrationTarget_.computeMarkerCornersFromPose(center, up, normal,
                                                    iReq.observation.marker_ids,
                                                    markerCorners);
    estimatedMarkerCorners_.push_back(markerCorners);

    //--- copy marker corners to output cloud
    estimatedMarkerCornersCloudPtrs_.push_back(nullptr);
    estimatedMarkerCornersCloudPtrs_.back().reset(new pcl::PointCloud<pcl::PointXYZI>());
    auto pMarkerCloud = estimatedMarkerCornersCloudPtrs_.back();
    for (uint m = 0; m < iReq.observation.marker_ids.size(); m++)
    {
        for (cv::Point3f corner : markerCorners[m])
        {
            //--- construct point for point cloud

            pcl::PointXYZI pclPnt;
            pclPnt.x         = static_cast<float>(corner.x);
            pclPnt.y         = static_cast<float>(corner.y);
            pclPnt.z         = static_cast<float>(corner.z);
            pclPnt.intensity = iReq.observation.marker_ids[m];

            pMarkerCloud->push_back(pclPnt);
        }
    }

    //--- target pose
    capturedCalibTargetPoses_.push_back(targetPose);

    //--- write response
    oRes.isAccepted = true;
    oRes.msg        = "Successfully added target observations through marker points. "
                      "Current number of observed target positions: " +
               std::to_string(capturedCalibTargetPoses_.size());
    ROS_INFO("[%s] %s",
             getLoggingId().c_str(),
             oRes.msg.c_str());

    return true;
}

//==================================================================================================
bool DataProcessor3d::onImportMarkerObservations(
  multisensor_calibration::ImportMarkerObservations::Request& iReq,
  multisensor_calibration::ImportMarkerObservations::Response& oRes)
{
    //--- reset data processor
    this->reset();

    //--- loop through observation list and add observations
    for (uint i = 0; i < iReq.observation_list.size(); ++i)
    {
        AddMarkerObservations addObs;
        addObs.request.observation = iReq.observation_list[i];
        onAddMarkerObservations(addObs.request, addObs.response);

        if (!addObs.response.isAccepted)
        {
            oRes.isAccepted = false;
            oRes.msg        = "Something went wrong when trying to add observation at index " +
                       std::to_string(i) + ".";

            ROS_ERROR("[%s] %s", getLoggingId().c_str(), oRes.msg.c_str());

            return false;
        }
    }

    oRes.isAccepted = true;
    oRes.msg        = "Successfully imported all marker observations.";
    ROS_INFO("[%s] %s",
             getLoggingId().c_str(),
             oRes.msg.c_str());

    return true;
}

//==================================================================================================
void DataProcessor3d::writeMarkerObservationsToFile(
  const fs::path& iFilePath,
  const std::vector<uint>& iMarkerIds,
  const std::vector<std::array<cv::Point3f, 4>>& iMarkerCorners)
{
    try
    {
        std::fstream observationsFout(iFilePath, std::ios_base::out);
        observationsFout << "# 3D points of the estimated marker corners of the detected "
                            "calibration target."
                         << std::endl;
        observationsFout << "#    Marker-ID Top-Left Top-Right Bottom-Right Bottom-Left"
                         << std::endl;
        for (uint m = 0; m < iMarkerIds.size(); ++m)
        {
            observationsFout << iMarkerIds[m];
            for (uint c = 0; c < 4; ++c)
            {
                observationsFout << " " << iMarkerCorners[m][c];
            }
            observationsFout << std::endl;
        }
        observationsFout.close();
    }
    catch (const std::exception& e)
    {
        throw e;
    }
}

//==================================================================================================
bool DataProcessor3d::readMarkerObservationsFromFile(
  const fs::path& iFilePath,
  std::vector<uint>& oMarkerIds,
  std::vector<std::array<cv::Point3f, 4>>& oMarkerCorners)
{

    if (!fs::exists(iFilePath))
        return false;

    try
    {
        std::fstream observationsFin(iFilePath, std::ios_base::in);

        std::string line;
        while (std::getline(observationsFin, line))
        {
            //--- get leading number as marker ID, this will also skip comment lines
            std::regex markerIdRegex(R"(^\s*(\d+))");
            std::smatch idMatch;
            if (std::regex_search(line, idMatch, markerIdRegex))
                oMarkerIds.push_back(static_cast<uint>(std::stoi(idMatch[1].str())));
            else
                continue;

            //--- get corner point in between the brackets
            std::regex cornerPointRegex(R"(\[([^\]]+)\])");
            std::smatch cornerMatch;
            std::array<cv::Point3f, 4> corners;
            int cornerItr = 0;

            std::string::const_iterator searchStart(line.cbegin());
            while (std::regex_search(searchStart, line.cend(), cornerMatch, cornerPointRegex))
            {
                std::string content = cornerMatch[1].str();

                std::stringstream ss(content);
                std::string numberStr;
                std::vector<float> pointCoords;
                while (std::getline(ss, numberStr, ','))
                {
                    pointCoords.push_back(std::stof(numberStr));
                }
                corners[cornerItr] = cv::Point3f(pointCoords[0],
                                                 pointCoords[1],
                                                 pointCoords[2]);
                cornerItr++;

                searchStart = cornerMatch.suffix().first; // Suche fortsetzen nach dem letzten Treffer
            }

            //--- if not four corners were read, remove marker id again and continue with next line
            if (cornerItr == 4)
            {
                oMarkerCorners.push_back(corners);
            }
            else
            {
                oMarkerIds.pop_back();
                continue;
            }
        }
    }
    catch (const std::exception& e)
    {
        throw e;
    }

    return true;
}

} // namespace multisensor_calibration