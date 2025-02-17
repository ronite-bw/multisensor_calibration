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

#ifndef MULTISENSORCALIBRATION_GUIDEDLIDARLIDARTARGETPLACEMENTNODELT_H
#define MULTISENSORCALIBRATION_GUIDEDLIDARLIDARTARGETPLACEMENTNODELT_H

// ROS
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/timer.h>

// OpenCV
#include <opencv2/core.hpp>

// multisensor_calibration
#include "../include/multisensor_calibration/common/common.h"
#include "GuidanceBase.h"

namespace multisensor_calibration
{

/**
 * @ingroup nodelets
 * @ingroup guidance
 * @brief Nodelet for guided target placement in the context of extrinsic lidar-lidar calibration.
 *
 * @note The guidance feature of multisensor_calibration is currently still in development and not yet
 * functional.
 *
 */
class GuidedLidarLidarTargetPlacementNodelet : public GuidanceBase, public nodelet::Nodelet
{

    //--- METHOD DECLARATION ---//
  public:
    /**
     * @brief Default constructor.
     */
    GuidedLidarLidarTargetPlacementNodelet();

    /**
     * @brief Destructor
     */
    virtual ~GuidedLidarLidarTargetPlacementNodelet();

  private:
    /**
     * @overload
     * @brief Method to compute overlapping Fov between the two sensors based on the estimated extrinsic
     * pose.
     *
     * @note This is a pure virtual method and needs to be implemented by the specific subclasses.
     */
    void computeExtrinsicFovBoundingPlanes() override;

    /**
     * @overload
     * @brief Method to compute Fov of the two sensors based on the intrinsic parameters.
     *
     * @note This is a pure virtual method and needs to be implemented by the specific subclasses.
     */
    bool computeIntrinsicFovBoundingPlanes() override;

    /**
     * @overload
     * @brief Method to compute next target pose.
     *
     * @note This is a pure virtual method and needs to be implemented by the specific subclasses.
     */
    void computeNextTargetPose() override;

    /**
     * @overload
     * @brief Method to initialize publishers
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    bool initializePublishers(ros::NodeHandle& ioNh) override;

    /**
     * @overload
     * @brief Method to initialize subscribers.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    bool initializeSubscribers(ros::NodeHandle& ioNh) override;

    /**
     * @overload
     * @brief Method to initialize timers.
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    bool initializeTimers(ros::NodeHandle& ioNh) override;

    /**
     * @overload
     * @brief The onInit method is called by nodelet manager. It is responsible for initializing the
     * nodelet.
     *
     * @note It is important that this method returns, otherwise the nodelet gets stuck during the
     * initialization
     */
    void onInit() override;

    /**
     * @brief Method to publish marker as guidance box at the next target position.
     */
    void publishGuidanceBox(const ros::TimerEvent&) const;

    /**
     * @overload
     * @brief Method to reset nextTargetPose_
     */
    void resetNextTargetPose() override;

    //--- MEMBER DECLARATION ---//

  protected:
    /// Node handle
    using GuidanceBase::nh_;

    /// Publisher for box guiding the next target position
    ros::Publisher guidanceBoxPub_;

    /// Timer object to trigger publishing of guidance box.
    ros::Timer publishGuidanceBoxTimer_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_GUIDEDLIDARLIDARTARGETPLACEMENTNODELT_H