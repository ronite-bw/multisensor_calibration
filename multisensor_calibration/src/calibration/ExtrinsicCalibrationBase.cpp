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

#include "../include/multisensor_calibration/calibration/ExtrinsicCalibrationBase.h"

// PCL
#include <pcl/common/io.h>

// multisensor_calibration
#include "../include/multisensor_calibration/common/utils.hpp"

namespace multisensor_calibration
{

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  ExtrinsicCalibrationBase(ECalibrationType type) :
  CalibrationBase(type),
  pSrcDataProcessor_(nullptr),
  pRefDataProcessor_(nullptr),
  srcSensorName_(""),
  srcTopicName_(""),
  srcFrameId_(""),
  refSensorName_(""),
  refTopicName_(""),
  refFrameId_(""),
  sensorExtrinsics_(std::vector<lib3d::Extrinsics>(1, lib3d::Extrinsics())),
  calibResult_(),
  useTfTreeAsInitialGuess_(false)
{
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::~ExtrinsicCalibrationBase()
{
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::initializeServices(
  ros::NodeHandle& ioNh)
{
    //--- call parent method
    bool isSuccessful = CalibrationBase::initializeServices(ioNh);

    //--- remove last observation
    removeObsSrv_ = ioNh.advertiseService(
      REMOVE_OBSERVATION_SRV_NAME,
      &ExtrinsicCalibrationBase::onRequestRemoveObservation, this);

    //--- calibration meta data
    calibMetaDataSrv_ = ioNh.advertiseService(
      REQUEST_META_DATA_SRV_NAME,
      &ExtrinsicCalibrationBase::onRequestCalibrationMetaData, this);

    //--- sensor extrinsics
    sensorExtrinsicsSrv_ = ioNh.advertiseService(
      REQUEST_SENSOR_EXTRINSICS_SRV_NAME,
      &ExtrinsicCalibrationBase::onRequestSensorExtrinsics, this);

    return isSuccessful;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::onRequestCalibrationMetaData(
  multisensor_calibration::CalibrationMetaData::Request& iReq,
  multisensor_calibration::CalibrationMetaData::Response& oRes)
{
    UNUSED_VAR(iReq);

    if (!isInitialized_)
        return false;

    oRes.calib_type = static_cast<int>(type_);

    oRes.robot_ws_path = (fs::exists(pRobotWs_->getPath()))
                           ? pRobotWs_->getPath().string()
                           : "";
    oRes.calib_ws_path = (fs::exists(pCalibrationWs_->getPath()))
                           ? pCalibrationWs_->getPath().string()
                           : "";

    oRes.calib_target_file_path = calibTargetFilePath_;

    oRes.src_sensor_name = srcSensorName_;
    oRes.src_topic_name  = srcTopicName_;
    oRes.src_frame_id    = srcFrameId_;

    oRes.ref_sensor_name = refSensorName_;
    oRes.ref_topic_name  = refTopicName_;
    oRes.ref_frame_id    = refFrameId_;

    oRes.base_frame_id = baseFrameId_;

    oRes.isComplete = (!oRes.robot_ws_path.empty() &&
                       !oRes.calib_ws_path.empty() &&
                       !oRes.calib_target_file_path.empty() &&
                       !oRes.src_sensor_name.empty() &&
                       !oRes.src_topic_name.empty() &&
                       !oRes.src_frame_id.empty() &&
                       !oRes.ref_sensor_name.empty() &&
                       !oRes.ref_topic_name.empty() &&
                       !oRes.ref_frame_id.empty());

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::onRequestSensorExtrinsics(
  multisensor_calibration::SensorExtrinsics::Request& iReq,
  multisensor_calibration::SensorExtrinsics::Response& oRes)
{
    //--- get empty or last sensor extrinsics and convert into tf::Transform
    lib3d::Extrinsics extrinsics = (sensorExtrinsics_.empty())
                                     ? lib3d::Extrinsics()
                                     : sensorExtrinsics_.back();

    tf::Transform ref2LocalTransform;
    utils::setTfTransformFromCameraExtrinsics(extrinsics, ref2LocalTransform);

    //--- transform ref2LocalTransform into request extrinsic type
    tf::StampedTransform tmpTransform;
    if (iReq.extrinsic_type == SensorExtrinsics::Request::SENSOR_2_SENSOR &&
        !baseFrameId_.empty())
    {
        try
        {
            //--- look up the transform from refFrameId to baseFrameId since, the inverse from
            //--- baseFrameId to refFrameId is required.
            tfListener_.lookupTransform(baseFrameId_, refFrameId_,
                                        ros::Time(0), tmpTransform);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("[%s]"
                      "\n\t> tf::TransformException: %s",
                      nodeletName_.c_str(), ex.what());
            return false;
        }

        ref2LocalTransform *= tmpTransform;
    }

    //--- construct response
    utils::cvtTfTransform2GeometryPose(ref2LocalTransform, oRes.extrinsics);

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  saveCalibrationSettingsToWorkspace()
{
    if (!CalibrationBase::saveCalibrationSettingsToWorkspace())
        return false;

    QSettings* pCalibSettings = pCalibrationWs_->settingsPtr();
    if (!pCalibSettings)
        return false;

    //--- base frame id
    pCalibSettings->setValue("calibration/base_frame_id",
                             QString::fromStdString(baseFrameId_));

    //--- initial guess
    pCalibSettings->setValue("calibration/use_initial_guess",
                             QVariant::fromValue(useTfTreeAsInitialGuess_));

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::readLaunchParameters(
  const ros::NodeHandle& iNh)
{
    if (!CalibrationBase::readLaunchParameters(iNh))
        return false;

    //--- base frame id
    baseFrameId_ =
      readStringLaunchParameter(iNh, "base_frame_id", "");

    //--- use TF-Tree as initial guess
    iNh.param<bool>("use_initial_guess", useTfTreeAsInitialGuess_, false);

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
template <typename Id_T, typename Obs_T, typename Cloud_T, typename Pose_T>
void ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  removeObservationsFromIteration(
    const uint& iCalibrationItr,
    std::set<Id_T>& ioIds,
    std::vector<Obs_T>& ioObs,
    std::vector<Cloud_T>& ioClouds,
    std::vector<Pose_T>& ioPoses) const
{
    typename std::set<Id_T>::iterator firstId, lastId;       // start and end position of ids to remove
    typename std::vector<Obs_T>::iterator firstObs, lastObs; // start and end position of obs to remove
    typename std::vector<Cloud_T>::iterator firstCloud;      // start position of cloud to remove
    typename std::vector<Pose_T>::iterator firstPose;        // start position of pose to remove
    bool isStartPosSet = false;
    bool isEndPosSet   = false;
    int startIdIdx, endIdIdx; // index of start and end element of ids to remove

    //--- find start and end position of ids that are to be removed.
    for (typename std::set<Id_T>::iterator idItr = ioIds.begin();
         idItr != ioIds.end();
         ++idItr)
    {
        //--- set start pos if id is part of calibration iteration and start pos has not yet been set
        if (*idItr >= (iCalibrationItr * 100) &&
            isStartPosSet == false)
        {
            firstId       = idItr;
            startIdIdx    = std::distance(ioIds.begin(), idItr);
            isStartPosSet = true;
        }
        //--- if idItr is part of next calibration iteration and start pos is set, set end pos
        else if (*idItr >= ((iCalibrationItr + 1) * 100) &&
                 isStartPosSet == true &&
                 isEndPosSet == false)
        {
            lastId      = idItr;
            endIdIdx    = std::distance(ioIds.begin(), idItr);
            isEndPosSet = true;
        }
    }

    //--- if at end of list and end pos is not yet set, set to end of list
    if (isEndPosSet == false)
    {
        lastId      = ioIds.end();
        endIdIdx    = std::distance(ioIds.begin(), ioIds.end());
        isEndPosSet = true;
    }

    //--- calculate start and end position of observation to be removed
    firstObs = ioObs.begin() + (startIdIdx * 4);
    lastObs  = ioObs.begin() + (endIdIdx * 4);

    //--- calculate start position of clouds to be removed
    firstCloud = ioClouds.begin() + startIdIdx;

    //--- calculate start position of poses to be removed
    firstPose = ioPoses.begin() + startIdIdx;

    //--- remove elements
    ioIds.erase(firstId, lastId);
    ioObs.erase(firstObs, lastObs);
    ioClouds.erase(firstCloud);
    ioPoses.erase(firstPose);
}
template void ExtrinsicCalibrationBase<CameraDataProcessor, LidarDataProcessor>::
  removeObservationsFromIteration<
    uint, cv::Point2f, pcl::PointCloud<pcl::PointXYZ>::Ptr, lib3d::Extrinsics>(
    const uint&, std::set<uint>&, std::vector<cv::Point2f>&,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<lib3d::Extrinsics>&) const;
template void ExtrinsicCalibrationBase<CameraDataProcessor, LidarDataProcessor>::
  removeObservationsFromIteration<
    uint, cv::Point3f, pcl::PointCloud<pcl::PointXYZ>::Ptr, lib3d::Extrinsics>(
    const uint&, std::set<uint>&, std::vector<cv::Point3f>&,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<lib3d::Extrinsics>&) const;
template void ExtrinsicCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>::
  removeObservationsFromIteration<
    uint, cv::Point2f, pcl::PointCloud<pcl::PointXYZ>::Ptr, lib3d::Extrinsics>(
    const uint&, std::set<uint>&, std::vector<cv::Point2f>&,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<lib3d::Extrinsics>&) const;
template void ExtrinsicCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>::
  removeObservationsFromIteration<
    uint, cv::Point3f, pcl::PointCloud<pcl::PointXYZ>::Ptr, lib3d::Extrinsics>(
    const uint&, std::set<uint>&, std::vector<cv::Point3f>&,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<lib3d::Extrinsics>&) const;
template void ExtrinsicCalibrationBase<LidarDataProcessor, LidarDataProcessor>::
  removeObservationsFromIteration<
    uint, cv::Point3f, pcl::PointCloud<pcl::PointXYZ>::Ptr, lib3d::Extrinsics>(
    const uint&, std::set<uint>&, std::vector<cv::Point3f>&,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<lib3d::Extrinsics>&) const;
template void ExtrinsicCalibrationBase<LidarDataProcessor, ReferenceDataProcessor3d>::
  removeObservationsFromIteration<
    uint, cv::Point3f, pcl::PointCloud<pcl::PointXYZ>::Ptr, lib3d::Extrinsics>(
    const uint&, std::set<uint>&, std::vector<cv::Point3f>&,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&, std::vector<lib3d::Extrinsics>&) const;

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
template <typename Id_T, typename Obs_T>
void ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  removeCornerObservationsWithoutCorrespondence(
    const std::set<Id_T>& iReferenceIds,
    std::set<Id_T>& ioSrcIds,
    std::vector<Obs_T>& ioSrcObs) const
{
    //--- loop over ioSrcIds and check if corresponding ID is also in
    //--- iReferenceIds
    //--- if not remove correspondence
    //--- since the sets are ordered, it can be iterated in parallel from front to back
    typename std::set<Id_T>::iterator refIdItr = // iterator of reference id list
      iReferenceIds.begin();
    typename std::set<Id_T>::iterator srcIdItr = // iterator of source id list
      ioSrcIds.begin();
    while (srcIdItr != ioSrcIds.end())
    {
        //--- if both ref ID and src ID are equal, increment iterators and continue
        if (*refIdItr == *srcIdItr)
        {
            ++refIdItr;
            ++srcIdItr;
            continue;
        }

        //--- compute index if src iterator and remove id and observations
        uint srcIdx        = std::distance(ioSrcIds.begin(), srcIdItr);
        auto eraseStartPos = ioSrcObs.begin() + (srcIdx * 4);
        ioSrcObs.erase(eraseStartPos, eraseStartPos + 4);
        srcIdItr = ioSrcIds.erase(srcIdItr);
    }
}
template void ExtrinsicCalibrationBase<CameraDataProcessor, LidarDataProcessor>::
  removeCornerObservationsWithoutCorrespondence<uint, cv::Point2f>(
    const std::set<uint>&, std::set<uint>&, std::vector<cv::Point2f>& ioSrcObs) const;
template void ExtrinsicCalibrationBase<CameraDataProcessor, LidarDataProcessor>::
  removeCornerObservationsWithoutCorrespondence<uint, cv::Point3f>(
    const std::set<uint>&, std::set<uint>&, std::vector<cv::Point3f>& ioSrcObs) const;
template void ExtrinsicCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>::
  removeCornerObservationsWithoutCorrespondence<uint, cv::Point2f>(
    const std::set<uint>&, std::set<uint>&, std::vector<cv::Point2f>& ioSrcObs) const;
template void ExtrinsicCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>::
  removeCornerObservationsWithoutCorrespondence<uint, cv::Point3f>(
    const std::set<uint>&, std::set<uint>&, std::vector<cv::Point3f>& ioSrcObs) const;
template void ExtrinsicCalibrationBase<LidarDataProcessor, LidarDataProcessor>::
  removeCornerObservationsWithoutCorrespondence<uint, cv::Point3f>(
    const std::set<uint>&, std::set<uint>&, std::vector<cv::Point3f>& ioSrcObs) const;
template void ExtrinsicCalibrationBase<LidarDataProcessor, ReferenceDataProcessor3d>::
  removeCornerObservationsWithoutCorrespondence<uint, cv::Point3f>(
    const std::set<uint>&, std::set<uint>&, std::vector<cv::Point3f>& ioSrcObs) const;

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
void ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::reset()
{
    //--- call method of parent class
    CalibrationBase::reset();

    sensorExtrinsics_ = std::vector<lib3d::Extrinsics>(1, lib3d::Extrinsics());
    calibResult_      = CalibrationResult();
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
void ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::saveCalibration()
{
    //--- save results to calibration workspace
    if (!saveResultsToCalibrationWorkspace())
    {
        ROS_WARN("[%s] Something went wrong while writing results to calibration workspace."
                 "\n\t> Workspace: %s",
                 nodeletName_.c_str(), pCalibrationWs_->getPath().string().c_str());
    }
    else
    {
        ROS_INFO("[%s] Writing results to calibration workspace: Successful!",
                 nodeletName_.c_str());
    }

    //--- save to URDF model
    if (isUrdfModelAvailable_)
    {
        if (!isFrameIdInUrdfModel(srcFrameId_))
        {
            ROS_WARN("[%s]"
                     "\n\t> Source Frame ID is not available as link in the URDF model file."
                     "\n\t> Results are not written to URDF model file."
                     "\n\t> Frame ID: %s",
                     nodeletName_.c_str(), srcFrameId_.c_str());
        }
        else if (!isFrameIdInUrdfModel((!baseFrameId_.empty()) ? baseFrameId_ : refFrameId_))
        {
            ROS_WARN("[%s]"
                     "\n\t> Base/Reference Frame ID is not available as link in the URDF model file."
                     "\n\t> Results are not written to URDF model file."
                     "\n\t> Frame ID: %s",
                     nodeletName_.c_str(),
                     (!baseFrameId_.empty()) ? baseFrameId_.c_str() : refFrameId_.c_str());
        }
        else if (!saveCalibrationToUrdfModel())
        {
            ROS_WARN("[%s]"
                     "\n\t> Something went wrong while writing results to URDF model file."
                     "\n\t> URDF model file: %s",
                     nodeletName_.c_str(), urdfModelPath_.string().c_str());
        }
        else
        {
            ROS_INFO("[%s] Writing results to URDF model file: Successful!",
                     nodeletName_.c_str());
        }
    }

    //--- save observations
    if (saveObservationsToWs_)
    {
        if (!saveObservationsToCalibrationWorkspace())
        {
            ROS_WARN("[%s] Something went wrong while writing observations to calibration "
                     "workspace.\n\t> Workspace: %s",
                     nodeletName_.c_str(), pCalibrationWs_->getPath().string().c_str());
        }
        else
        {
            ROS_INFO("[%s] Writing observations to calibration workspace: Successful!",
                     nodeletName_.c_str());
        }
    }
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::saveCalibrationToUrdfModel()
{
    // Pointer to the XML element of the desired joint node
    tinyxml2::XMLElement* pJointElement = nullptr;

    // Pointer to the XML element of the child node of the desired joint, i.e. source sensor
    tinyxml2::XMLElement* pChildElement = nullptr;

    // Pointer to the XML element of the parent node of the desired joint, i.e. reference sensor
    tinyxml2::XMLElement* pParentElement = nullptr;

    // Pointer to the XML element of the origin node of the desired joint, i.e. transformation between reference and source
    tinyxml2::XMLElement* pOriginElement = nullptr;

    // Indicator if data has been inserted into xml doc
    std::vector<bool> isDataInserted = std::vector<bool>(calibResult_.calibrations.size(), false);

    for (uint i = 0; i < calibResult_.calibrations.size(); ++i)
    {
        //--- loop over <joint> nodes within urdf XML file
        pJointElement = urdfModelDoc_.RootElement()->FirstChildElement("joint");
        while (pJointElement && !isDataInserted[i])
        {
            //--- loop over <child> nodes within joint
            pChildElement = pJointElement->FirstChildElement("child");
            while (pChildElement && !isDataInserted[i])
            {
                //--- if the link 'link' attribute of the <child> has the same name as the source sensor,
                //--- get <parent> and <origin> node
                if (pChildElement->Attribute("link") == calibResult_.calibrations[i].srcFrameId)
                {
                    pParentElement = pJointElement->FirstChildElement("parent");
                    pOriginElement = pJointElement->FirstChildElement("origin");

                    //--- write result into xml tree
                    if (pParentElement != nullptr && pOriginElement != nullptr)
                    {
                        pParentElement->SetAttribute(
                          "link", (calibResult_.calibrations[i].baseFrameId.empty())
                                    ? calibResult_.calibrations[i].refFrameId.c_str()
                                    : calibResult_.calibrations[i].baseFrameId.c_str());

                        std::stringstream xyzStrStream;
                        xyzStrStream << calibResult_.calibrations[i].XYZ.x() << " "
                                     << calibResult_.calibrations[i].XYZ.y() << " "
                                     << calibResult_.calibrations[i].XYZ.z();
                        pOriginElement->SetAttribute("xyz", xyzStrStream.str().c_str());

                        std::stringstream rpyStrStream;
                        rpyStrStream << calibResult_.calibrations[i].RPY.x() << " "
                                     << calibResult_.calibrations[i].RPY.y() << " "
                                     << calibResult_.calibrations[i].RPY.z();
                        pOriginElement->SetAttribute("rpy", rpyStrStream.str().c_str());

                        isDataInserted[i] = true;
                    }
                }
                pChildElement = pChildElement->NextSiblingElement("child");
            }
            pJointElement = pJointElement->NextSiblingElement("joint");
        }
    }

    for (auto success : isDataInserted)
        if (!success)
            return false;

    //--- backup urdf model file
    utils::backupFile(urdfModelPath_);

    //--- save new XML Document
    urdfModelDoc_.SaveFile(urdfModelPath_.c_str());

    //--- reload URDF Model
    urdfModel_.clear();
    urdfModel_.initXml(&urdfModelDoc_);

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  saveObservationsToCalibrationWorkspace() const
{
    bool isSuccessful = true;

    isSuccessful &= pSrcDataProcessor_->saveObservations(
      pCalibrationWs_->getPath().append(OBSERVATIONS_SUBDIR_NAME));
    isSuccessful &= pRefDataProcessor_->saveObservations(
      pCalibrationWs_->getPath().append(OBSERVATIONS_SUBDIR_NAME));

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  saveResultsToCalibrationWorkspace() const
{
    // lambda function to write content to file
    auto writeToFile = [&](const std::string& filename, const std::string& content) -> bool
    {
        // absolute path to results file
        fs::path resultsFilePath = pCalibrationWs_->getPath().append(filename);

        //--- write results to file
        std::fstream ofStream(resultsFilePath, std::ios_base::out);
        if (ofStream.is_open())
        {
            ofStream << content;
            ofStream.close();
        }
        else
        {
            return false;
        }

        return true;
    };

    bool isSuccesful = true;

    isSuccesful &= writeToFile(CALIB_RESULTS_FILE_NAME, calibResult_.toString());
    isSuccesful &= writeToFile(URDF_SNIPPET_FILE_NAME, calibResult_.urdfSnippet());

    return isSuccesful;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
bool ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  setSensorExtrinsicsFromFrameIds(const std::string& iSourceFrameId,
                                  const std::string& iReferenceFrameId)
{
    //--- initialize extrinsics
    tf::StampedTransform transform;
    if (tfListener_.frameExists(iSourceFrameId) &&
        tfListener_.frameExists(iReferenceFrameId))
    {
        try
        {
            tfListener_.lookupTransform(iSourceFrameId, iReferenceFrameId,
                                        ros::Time(0), transform);
            utils::setCameraExtrinsicsFromTfTransform(transform,
                                                      sensorExtrinsics_.back());
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("[%s]"
                      "\n\t> tf::TransformException: %s",
                      nodeletName_.c_str(), ex.what());
            return false;
        }
    }
    else
    {
        ROS_WARN("[%s]"
                 "\n\t> Frame %s or frame %s does not exists! "
                 "Initializing extrinsic transformation with null rotation and translation.",
                 nodeletName_.c_str(), iSourceFrameId.c_str(), iReferenceFrameId.c_str());
        sensorExtrinsics_.back() = lib3d::Extrinsics();
        return false;
    }

    return true;
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
std::pair<tf::Vector3, tf::Vector3> ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  computeTargetPoseStdDev(
    const std::vector<lib3d::Extrinsics>& iSrcTargetPoses,
    const std::vector<lib3d::Extrinsics>& iRefTargetPoses) const
{
    //--- get min num overlapping target poses
    const uint MIN_TARGET_POSES = std::min(iSrcTargetPoses.size(), iRefTargetPoses.size());

    //--- if only one overlapping target pose is available, return maximum deviation
    if (MIN_TARGET_POSES <= 1)
    {
        return std::make_pair(tf::Vector3(FLT_MAX, FLT_MAX, FLT_MAX),
                              tf::Vector3(FLT_MAX, FLT_MAX, FLT_MAX));
    }

    //--- get RT Matrix of last extrinsic pose to transform source sensor into reference
    const cv::Matx44d EXTR_TRANSF_MAT = sensorExtrinsics_.back().getRTMatrix(
      lib3d::Extrinsics::LOCAL_2_REF);

    // list of differences in xyz and rpy of target poses
    std::vector<tf::Vector3> xyz_differences, rpy_differences;

    // mean values of differences in xyz of target poses
    tf::Vector3 xyz_difference_mean = tf::Vector3(0, 0, 0);

    // mean values of differences in rpy of target poses
    tf::Vector3 rpy_difference_mean = tf::Vector3(0, 0, 0);

    //--- loop over number of overlapping target poses
    for (uint i = 0; i < MIN_TARGET_POSES; ++i)
    {
        //--- transform source target pose into frame of the reference sensor based on the last
        //--- computed extrinsics
        lib3d::Extrinsics transSrcPose(lib3d::Extrinsics::LOCAL_2_REF);
        transSrcPose.setRTMatrix(
          EXTR_TRANSF_MAT * iSrcTargetPoses[i].getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF));

        // tf::Transform of the transformed source pose and the ref pose.
        tf::Transform transSrcPoseTransform, refPoseTransform;

        //--- get tf::Transforms from lib3d::Extrinsics
        utils::setTfTransformFromCameraExtrinsics(transSrcPose, transSrcPoseTransform);
        utils::setTfTransformFromCameraExtrinsics(iRefTargetPoses[i], refPoseTransform);

        //--- compute and push back difference in XYZ
        xyz_differences.push_back(transSrcPoseTransform.getOrigin() - refPoseTransform.getOrigin());

        //--- compute and push back difference in RPY
        double sRoll, sPitch, sYaw, rRoll, rPitch, rYaw;
        transSrcPoseTransform.getBasis().getRPY(sRoll, sPitch, sYaw);
        refPoseTransform.getBasis().getRPY(rRoll, rPitch, rYaw);
        rpy_differences.push_back(tf::Vector3(sRoll - rRoll, sPitch - rPitch, sYaw - rYaw));

        //--- add previously computed difference to running mean of XYZ and RPY
        xyz_difference_mean += xyz_differences.back();
        rpy_difference_mean += rpy_differences.back();
    }

    //--- normalize running mean values
    xyz_difference_mean /= MIN_TARGET_POSES;
    rpy_difference_mean /= MIN_TARGET_POSES;

    tf::Vector3 xyz_var = tf::Vector3(0, 0, 0); // variance in XYZ of pose
    tf::Vector3 rpy_var = tf::Vector3(0, 0, 0); // variance in RPY of pose

    //--- compute variance
    for (uint i = 0; i < MIN_TARGET_POSES; ++i)
    {
        xyz_var += tf::Vector3(std::pow(xyz_differences[i].x() - xyz_difference_mean.x(), 2.f),
                               std::pow(xyz_differences[i].y() - xyz_difference_mean.y(), 2.f),
                               std::pow(xyz_differences[i].z() - xyz_difference_mean.z(), 2.f));

        rpy_var += tf::Vector3(std::pow(rpy_differences[i].x() - rpy_difference_mean.x(), 2.f),
                               std::pow(rpy_differences[i].y() - rpy_difference_mean.y(), 2.f),
                               std::pow(rpy_differences[i].z() - rpy_difference_mean.z(), 2.f));
    }
    xyz_var /= MIN_TARGET_POSES;
    rpy_var /= MIN_TARGET_POSES;

    //--- compute and return standard deviation
    return std::make_pair(
      tf::Vector3(std::sqrt(xyz_var.x()), std::sqrt(xyz_var.y()), std::sqrt(xyz_var.z())),
      tf::Vector3(lib3d::radianToDegree(std::sqrt(rpy_var.x())),
                  lib3d::radianToDegree(std::sqrt(rpy_var.y())),
                  lib3d::radianToDegree(std::sqrt(rpy_var.z()))));
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
std::string ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::CalibrationResult::
  toString() const
{
    std::stringstream strStream;

    for (auto calib : calibrations)
    {
        strStream << "Transformation from";
        if (!calib.baseFrameId.empty())
            strStream << "\n  base frame (Frame ID: " << calib.baseFrameId << ") as parent";
        else
            strStream << "\n  '" << calib.refSensorName
                      << "' (Frame ID: " << calib.refFrameId << ") as parent";
        strStream << "\nto";
        strStream << "\n  '" << calib.srcSensorName
                  << "' (Frame ID: " << calib.srcFrameId << ") as child:";
        strStream << "\n\t> XYZ: "
                  << calib.XYZ.x() << " " << calib.XYZ.y() << " " << calib.XYZ.z();
        strStream << "\n\t> RPY: "
                  << calib.RPY.x() << " " << calib.RPY.y() << " " << calib.RPY.z();
        strStream << "\n- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n";
    }
    strStream << "\nNumber of observations: " << numObservations;
    strStream << std::setprecision(4) << std::fixed
              << "\n"
              << error.first << ": " << error.second;

    strStream << std::setprecision(4) << std::fixed
              << "\nDeviation in poses of calibration target"
              << "\nwhen transformed between sensor frames:";

    strStream << "\n\t> XYZ (in m): ";
    if (!std::isnan(target_poses_stdDev.first.length()))
    {
        strStream << target_poses_stdDev.first.x() << " "
                  << target_poses_stdDev.first.y() << " "
                  << target_poses_stdDev.first.z();
    }
    else
    {
        strStream << "n/a";
    }

    strStream << "\n\t> RPY (in Deg.): ";
    if (!std::isnan(target_poses_stdDev.second.length()))
    {
        strStream << target_poses_stdDev.second.x() << " "
                  << target_poses_stdDev.second.y() << " "
                  << target_poses_stdDev.second.z();
    }
    else
    {
        strStream << "n/a";
    }

    return strStream.str();
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
std::string ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::CalibrationResult::
  urdfSnippet() const
{
    std::stringstream strStream;
    for (auto calib : calibrations)
    {
        strStream << "<joint name=\"" << calib.srcSensorName << "_joint\" type=\"fixed\">";
        strStream << "\n\t<parent link=\""
                  << ((!calib.baseFrameId.empty()) ? calib.baseFrameId : calib.refFrameId)
                  << "\"/>";
        strStream << "\n\t<child link=\"" << calib.srcFrameId << "\"/>";
        strStream << "\n\t<origin xyz=\""
                  << calib.XYZ.x() << " " << calib.XYZ.y() << " " << calib.XYZ.z() << "\" "
                  << "rpy=\""
                  << calib.RPY.x() << " " << calib.RPY.y() << " " << calib.RPY.z() << "\"/>";
        strStream << "\n</joint>";
        strStream << "\n<link name=\"" << calib.srcFrameId << "\">";
        strStream << "\n</link>";

        strStream << ((calibrations.size() > 1) ? "\n\n" : "");
    }

    return strStream.str();
}

template class ExtrinsicCalibrationBase<CameraDataProcessor, LidarDataProcessor>;
template class ExtrinsicCalibrationBase<CameraDataProcessor, ReferenceDataProcessor3d>;
template class ExtrinsicCalibrationBase<LidarDataProcessor, LidarDataProcessor>;
template class ExtrinsicCalibrationBase<LidarDataProcessor, ReferenceDataProcessor3d>;

} // namespace multisensor_calibration