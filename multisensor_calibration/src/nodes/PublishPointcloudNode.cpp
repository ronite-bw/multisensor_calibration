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

// Std
#include <filesystem>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#if !defined(TARGET_NAME)
#define TARGET_NAME ""
#endif

using namespace pcl::io;

const std::string DEFAULT_TOPIC_NAME = "cloud";
const std::string DEFAULT_FRAME_ID   = "map";

/**
 * @brief Entry point for the 'point_cloud_publisher' node.
 *
 * This will load a point cloud from a specified PLY file and publish it on the given topic with
 * the given frame_id.
 *
 * <b>Launch-Parameters:</b>
 * - ```point_cloud_file```: PLY file from which to load the point cloud.
 *    - Type: String
 *    - Default: ""
 * - ```topic_name```: Topic name on which to publish the point cloud.
 *    - Type: String
 *    - Default: ""
 * - ```frame_id```: Frame ID at which to publish the point cloud.
 *    - Type: String
 *    - Default: ""
 */
int main(int argc, char** argv)
{
    //---Initialize ROS node
    ros::init(argc, argv, TARGET_NAME);
    ros::NodeHandle nh("~");

    //--- Launch Parameters
    std::string pointcloudFileStr;
    std::string topicName;
    std::string frameId;

    //--- Get the launch parameters
    nh.param<std::string>("point_cloud_file", pointcloudFileStr, "");
    nh.param<std::string>("topic_name", topicName, DEFAULT_TOPIC_NAME);
    nh.param<std::string>("frame_id", frameId, DEFAULT_FRAME_ID);

    //--- sanity check for parameters
    std::filesystem::path pointcloudFilePath(pointcloudFileStr);
    if (pointcloudFilePath.is_relative())
    {
        pointcloudFilePath = std::filesystem::current_path();
        pointcloudFilePath /= pointcloudFileStr;
    }

    if (pointcloudFilePath.extension() != ".ply")
    {
        ROS_ERROR("[%s] 'pointcloud_file' does not end with '.ply'", TARGET_NAME);
        return 1;
    }
    if (!std::filesystem::exists(pointcloudFilePath))
    {
        ROS_ERROR("[%s] 'pointcloud_file' does not exits. File path: %s",
                  TARGET_NAME, pointcloudFilePath.c_str());
        return 1;
    }
    if (topicName.empty())
    {
        ROS_ERROR("[%s] 'topic_name' is empty. Setting to default: %s",
                  TARGET_NAME, DEFAULT_TOPIC_NAME.c_str());
        return 1;
    }
    if (frameId.empty())
    {
        ROS_ERROR("[%s] 'frame_id' is empty. Setting to default: %s",
                  TARGET_NAME, DEFAULT_FRAME_ID.c_str());
        return 1;
    }

    //--- Publisher for the Point Cloud
    ros::Publisher publisher = nh.advertise<sensor_msgs::PointCloud2>(topicName, 10);

    //--- Load the pointcloud from the .ply file
    pcl::PointCloud<pcl::PointXYZ> cloud;
    loadPLYFile<pcl::PointXYZ>(pointcloudFilePath, cloud);

    //-- Set the publish rate to 10Hz
    ros::Rate rate(10);

    while (ros::ok())
    {
        //--- Convert the Pointcloud to ROS msg type
        sensor_msgs::PointCloud2 pointcloud_msg;
        toROSMsg(cloud, pointcloud_msg);

        //--- set frame IS and time stamp to current time
        pointcloud_msg.header.frame_id = frameId;
        pointcloud_msg.header.stamp    = ros::Time::now();

        //---Publish the Pointcloud message
        publisher.publish(pointcloud_msg);

        //--- wait for the next iteration
        rate.sleep();
    }
}