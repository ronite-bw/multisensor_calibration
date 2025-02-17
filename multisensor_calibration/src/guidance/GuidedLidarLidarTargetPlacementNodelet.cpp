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

#include "../include/multisensor_calibration/guidance/GuidedLidarLidarTargetPlacementNodelet.h"

// Std
#define USE_MATH_DEFINES
#include <cmath>

// ROS
#include <pluginlib/class_list_macros.h>

// multisensor_calibration
#include "../../include/multisensor_calibration/common/utils.hpp"

namespace multisensor_calibration
{

//==================================================================================================
GuidedLidarLidarTargetPlacementNodelet::GuidedLidarLidarTargetPlacementNodelet() :
  GuidanceBase()
{
    resetNextTargetPose();
}

//==================================================================================================
GuidedLidarLidarTargetPlacementNodelet::~GuidedLidarLidarTargetPlacementNodelet()
{
}

//==================================================================================================
void GuidedLidarLidarTargetPlacementNodelet::computeExtrinsicFovBoundingPlanes()
{
    // TODO: implement
}

//==================================================================================================
bool GuidedLidarLidarTargetPlacementNodelet::computeIntrinsicFovBoundingPlanes()
{
    // TODO: implement

    return true;
}

//==================================================================================================
void GuidedLidarLidarTargetPlacementNodelet::computeNextTargetPose()
{
    // TODO: implement
}

//==================================================================================================
bool GuidedLidarLidarTargetPlacementNodelet::initializePublishers(ros::NodeHandle& ioNh)
{
    //--- advertise topic of guidance box
    guidanceBoxPub_ = ioNh.advertise<TargetPlacementBox_Message_T>(PLACEMENT_GUIDANCE_TOPIC_NAME, 10);

    return true;
}

//==================================================================================================
bool GuidedLidarLidarTargetPlacementNodelet::initializeSubscribers(ros::NodeHandle& ioNh)
{
    //--- call parent method
    bool isSuccessful = GuidanceBase::initializeSubscribers(ioNh);
    if (!isSuccessful)
        return false;

    return true;
}

//==================================================================================================
bool GuidedLidarLidarTargetPlacementNodelet::initializeTimers(ros::NodeHandle& ioNh)
{
    bool isSuccessful = GuidanceBase::initializeTimers(ioNh);
    if (!isSuccessful)
        return false;

    //--- initialize trigger to call routine to load the robot workspace
    publishGuidanceBoxTimer_ = ioNh.createTimer(
      ros::Duration(1), &GuidedLidarLidarTargetPlacementNodelet::publishGuidanceBox,
      this, false, true);

    return true;
}

//==================================================================================================
void GuidedLidarLidarTargetPlacementNodelet::onInit()
{
    //--- get global and private node handle
    nh_  = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    //--- set nodelet name and parent namespace
    nodeletName_     = Nodelet::getName();
    parentNamespace_ = nodeletName_.substr(0, nodeletName_.find_last_of('/'));

    //--- read launch parameters
    isInitialized_ &= readLaunchParameters(pnh_);

    //--- initialize publishers
    isInitialized_ &= initializePublishers(pnh_);

    //--- initialize services
    isInitialized_ &= initializeServices(pnh_);

    //--- initialize timers
    isInitialized_ &= initializeTimers(nh_);

    //--- start ros event loop
    if (isInitialized_)
    {
        calibMetaDataTimer_.start();
    }
}

//==================================================================================================
void GuidedLidarLidarTargetPlacementNodelet::publishGuidanceBox(const ros::TimerEvent&) const
{
    visualization_msgs::Marker markerMsg;

    //--- header
    markerMsg.header.frame_id = calibrationMetaData_.ref_frame_id;
    markerMsg.header.stamp    = ros::Time::now();

    //--- Set the namespace and id for this marker.  This serves to create a unique ID
    //--- Any marker sent with the same namespace and id will overwrite the old one
    markerMsg.ns = "target_placement_box";
    markerMsg.id = 0;

    //--- Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    markerMsg.type = visualization_msgs::Marker::CUBE;

    //--- Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    markerMsg.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    tf::Transform transform;
    utils::setTfTransformFromCameraExtrinsics(nextTargetPose_, transform);
    transform = transform.inverse(); // invert to get absolute pose relative to reference
    utils::cvtTfTransform2GeometryPose(transform, markerMsg.pose);

    //--- Set the scale of the marker -- 1x1x1 here means 1m on a side
    markerMsg.scale.x = (1.1 * calibrationTarget_.boardSize.width);
    markerMsg.scale.y = (1.1 * calibrationTarget_.boardSize.height);
    markerMsg.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    markerMsg.color.r = 0.0f;
    markerMsg.color.g = 0.0f;
    markerMsg.color.b = 1.0f;
    markerMsg.color.a = 0.6f;

    markerMsg.lifetime = ros::Duration();

    guidanceBoxPub_.publish(markerMsg);
}

//==================================================================================================
void GuidedLidarLidarTargetPlacementNodelet::resetNextTargetPose()
{
    nextTargetPose_.setTransfDirection(lib3d::Extrinsics::LOCAL_2_REF);
    nextTargetPose_.setTranslationVec(0, -3, 0);
    nextTargetPose_.setRotationMat(lib3d::Rotation::createRotationX_deg(90) *
                                   lib3d::Rotation::createRotationY_deg(30));
}

} // namespace multisensor_calibration

// Export the class as a plugin using the PLUGINLIB_EXPORT_CLASS macro.
PLUGINLIB_EXPORT_CLASS(multisensor_calibration::GuidedLidarLidarTargetPlacementNodelet, nodelet::Nodelet)