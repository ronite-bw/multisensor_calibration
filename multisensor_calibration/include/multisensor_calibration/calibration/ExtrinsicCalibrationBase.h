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

#ifndef MULTISENSORCALIBRATION_EXTRINSICCALIBRATIONBASE_H
#define MULTISENSORCALIBRATION_EXTRINSICCALIBRATIONBASE_H

// Std
#include <memory>
#include <string>
#include <tuple>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// multisensor_calibration
#include "../data_processing/CameraDataProcessor.h"
#include "../data_processing/LidarDataProcessor.h"
#include "../data_processing/ReferenceDataProcessor3d.h"
#include "CalibrationBase.h"
#include <multisensor_calibration/CalibrationMetaData.h>
#include <multisensor_calibration/RemoveLastObservation.h>
#include <multisensor_calibration/SensorExtrinsics.h>

namespace multisensor_calibration
{

/**
 * @ingroup calibration
 * @brief Base class of all extrinsic calibration nodelets.
 *
 * @tparam SrcDataProcessorT Class to process data from source sensor.
 * @tparam RefDataProcessorT Class to process data from reference sensor.
 */
template <class SrcDataProcessorT, class RefDataProcessorT>
class ExtrinsicCalibrationBase : public CalibrationBase
{
    //--- STRUCTS ---//

    /**
     * @ingroup calibration
     * @brief Struct holding extrinsic calibration data as part of the calibration result
     */
    struct ExtrinsicCalibration
    {
        /// Name of source sensor.
        std::string srcSensorName;

        /// Frame id of source sensor.
        std::string srcFrameId;

        /// Name of reference sensor.
        std::string refSensorName;

        /// Frame id of reference sensor.
        std::string refFrameId;

        /// Base frame id.
        std::string baseFrameId;

        /// Translation of extrinsic transformation.
        tf::Vector3 XYZ;

        /// Roll, Pitch, Yaw of extrinsic transformation.
        tf::Vector3 RPY;
    };

    /**
     * @ingroup calibration
     * @brief Calibration result for extrinsic calibrations.
     */
    struct CalibrationResult
    {
        /// Data of pairwise sensor calibration
        std::vector<ExtrinsicCalibration> calibrations;

        /// Number of target observations
        int numObservations;

        /// Error of calibration made up of a string name and the actual error value
        std::pair<std::string, double> error;

        /// Standard Deviation in XYZ and RPY of target poses when transformed between sensor
        /// frames. In this, the RPY angles are given in degrees.
        std::pair<tf::Vector3, tf::Vector3> target_poses_stdDev;

        /// Print out calibration results to string.
        std::string toString() const;

        /// Print URDF snippet of the calibration to string.
        std::string urdfSnippet() const;

        CalibrationResult()
        {
            calibrations.resize(1);

            target_poses_stdDev =
              std::make_pair(tf::Vector3(NAN, NAN, NAN),
                             tf::Vector3(NAN, NAN, NAN));
        }
    };

    //--- METHOD DECLARATION ---/
  public:
    /**
     * @brief Default constructor is deleted.
     */
    ExtrinsicCalibrationBase() = delete;

    /**
     * @brief Initialization constructor.
     *
     * @brief[in] type type of calibration.
     */
    ExtrinsicCalibrationBase(ECalibrationType type);

    /**
     * @brief Destructor
     */
    virtual ~ExtrinsicCalibrationBase();

  protected:
    using CalibrationBase::createAndStartCalibrationWorkflow;

    /**
     * @brief Method to initialize services. This overrides the CalibrationBase::initializeServices.
     * In this, the method from the base class is also called.
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    virtual bool initializeServices(ros::NodeHandle& ioNh);

    /**
     * @brief Handle call requesting calibration meta data.
     *
     * @param[in] iReq Request
     * @param[out] oRes Response.
     */
    bool onRequestCalibrationMetaData(
      multisensor_calibration::CalibrationMetaData::Request& iReq,
      multisensor_calibration::CalibrationMetaData::Response& oRes);

    /**
     * @brief Handle call requesting removal of last observation. This is a pure virtual definition
     * and needs to be implemented in a subclass.
     *
     * @param[in] iReq Request, with flag to capture calibration target
     * @param[out] oRes Response, empty.
     */
    virtual bool onRequestRemoveObservation(
      multisensor_calibration::RemoveLastObservation::Request& iReq,
      multisensor_calibration::RemoveLastObservation::Response& oRes) = 0;

    /**
     * @brief Handling call requesting sensor extrinsics.
     *
     * @param[in] iReq Request, empty.
     * @param[out] oRes Response.
     */
    virtual bool onRequestSensorExtrinsics(
      multisensor_calibration::SensorExtrinsics::Request& iReq,
      multisensor_calibration::SensorExtrinsics::Response& oRes);

    /**
     * @brief Save calibration settings to setting.ini inside calibration workspace.
     *
     * This overrides CalibrationBase::saveCalibrationSettingsToWorkspace. In this, the method of
     * the parent class is also called.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    bool saveCalibrationSettingsToWorkspace() override;

    /**
     * @brief Read launch parameters.
     *
     * This overrides CalibrationBase::readLaunchParameters. In this, the method of
     * the parent class is also called.
     *
     * The implementation within this class hold launch parameters that are common to all
     * calibration nodelets, e.g. robot_ws_path, target_config_file.
     *
     * @param[in] iNh Object of node handle
     * @return True if successful. False, otherwise (e.g. if sanity check fails)
     */
    bool readLaunchParameters(const ros::NodeHandle& iNh) override;

    /**
     * @brief Method to remove all observations that were captured during the given calibration
     * iteration.
     *
     * @tparam Id_T Type of IDs.
     * @tparam Obs_T Type of Observations.
     * @tparam Cloud_T Type of target clouds.
     * @tparam Pose_T Type of target board poses.
     * @param[in] iCalibrationItr Calibration iteration.
     * @param[in, out] ioIds List of IDs.
     * @param[in, out] ioObs List of Observations.
     * @param[in, out] ioClouds List of target clouds.
     * @param[in, out] ioPoses List of target board clouds.
     */
    template <typename Id_T, typename Obs_T, typename Cloud_T, typename Pose_T>
    void removeObservationsFromIteration(const uint& iCalibrationItr,
                                         std::set<Id_T>& ioIds,
                                         std::vector<Obs_T>& ioObs,
                                         std::vector<Cloud_T>& ioClouds,
                                         std::vector<Pose_T>& ioPoses) const;

    /**
     * @brief Method to remove observations from ta given source list that do not have
     * corresponding observations with the same ID in the reference list.
     *
     * @tparam Id_T Type of IDs
     * @tparam Obs_T Type of Observations
     * @param[in] iReferenceIds Reference list of IDs to check against.
     * @param[in, out] ioSrcIds List of IDs which are to be checked against the reference list.
     * @param[in, out] ioSrcObs List of Observations.
     */
    template <typename Id_T, typename Obs_T>
    void removeCornerObservationsWithoutCorrespondence(const std::set<Id_T>& iReferenceIds,
                                                       std::set<Id_T>& ioSrcIds,
                                                       std::vector<Obs_T>& ioSrcObs) const;

    /**
     * @brief Reset calibration.
     *
     * This overrides CalibrationBase::reset. In this, the method of
     * the parent class is also called.
     */
    void reset() override;

    /**
     * @brief Save calibration.
     *
     * This overrides CalibrationBase::saveCalibration. In this, the method of
     * the parent class is also called.
     */
    void saveCalibration() override;

    /**
     * @brief Method to save the calibration into the URDF model.
     *
     * This will write out the calibration into the provided urdf model. Prior to that the old
     * model file is copied and renamed as a backup.
     */
    bool saveCalibrationToUrdfModel();

    /**
     * @brief Trigger data processors to save observations that have been used for calibration to
     * the corresponding workspace.
     *
     * @return True, if successful. False, otherwise.
     */
    bool saveObservationsToCalibrationWorkspace() const;

    /**
     * @brief Method to save the results of the calibration into file in the calibration workspace.
     *
     * This will write out a file in the calibration workspace that olds teh calibration results.
     * If the desired file already exists, the existing file is copied and renamed.
     *
     * @return True, if successful. False, otherwise.
     */
    bool saveResultsToCalibrationWorkspace() const;

    /**
     * @brief Set relative sensor extrinsics (sensorExtrinsics_) from the given frame IDs.
     *
     * This will extract a transformation from the tfListener and set the sensor extrinsics
     * accordingly.
     *
     * @param[in] iSourceFrameId Frame Id of source sensor.
     * @param[in] iReferenceFrameId Frame Id of reference sensor.
     * @return True, if successful. False, otherwise.
     */
    bool setSensorExtrinsicsFromFrameIds(const std::string& iSourceFrameId,
                                         const std::string& iReferenceFrameId);

    /**
     * @brief Compute standard deviation in detected target poses when they are transformed between
     * the sensor frames based on the estimated extrinsic calibration.
     *
     * This will transform the detected target poses from the source sensor into the frame of the
     * reference sensor and compute the standard deviation between the poses. The transformation
     * is done using the sensor extrinsic that is estimated last.
     *
     * @param[in] iSrcTargetPoses Target poses detected in source sensor.
     * @param[in] iRefTargetPoses Target poses detected in reference sensor.
     * @return The standard deviation of the target poses in XYZ (in meters) and RPY (in degrees).
     */
    std::pair<tf::Vector3, tf::Vector3> computeTargetPoseStdDev(
      const std::vector<lib3d::Extrinsics>& iSrcTargetPoses,
      const std::vector<lib3d::Extrinsics>& iRefTargetPoses) const;

    //--- MEMBER DECLARATION ---/

  protected:
    /// Server to provide service to remove last observation
    ros::ServiceServer removeObsSrv_;

    /// Server to provide service to request calibration meta data
    ros::ServiceServer calibMetaDataSrv_;

    /// Server to provide service to request sensor extrinsics
    ros::ServiceServer sensorExtrinsicsSrv_;

    /// Pointer to data processor of source sensor.
    std::shared_ptr<SrcDataProcessorT> pSrcDataProcessor_;

    /// Pointer to data processor of reference sensor.
    std::shared_ptr<RefDataProcessorT> pRefDataProcessor_;

    /// Name of the source sensor.
    std::string srcSensorName_;

    /// Topic name of the data from the source sensor
    std::string srcTopicName_;

    /// Frame id of data from the source sensor
    std::string srcFrameId_;

    /// Name of the reference sensor.
    std::string refSensorName_;

    /// Topic name of the data from the source sensor
    std::string refTopicName_;

    /// Frame id of data from the reference sensor
    std::string refFrameId_;

    /// Frame id of base frame with respect to which the source frame is to be calibrated.
    /// If omitted, the source frame will be calibrated with respect to refFrameID_.
    std::string baseFrameId_;

    /// Object holding extrinsic transformation between the source and the reference sensor.
    std::vector<lib3d::Extrinsics> sensorExtrinsics_;

    /// Object holding the result of the calibration.
    CalibrationResult calibResult_;

    /// Fag to use frame IDs in TF-Tree as initial guess. Default: false.
    bool useTfTreeAsInitialGuess_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_EXTRINSICCALIBRATIONBASE_H