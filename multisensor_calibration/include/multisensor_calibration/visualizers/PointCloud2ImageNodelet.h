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

#ifndef MULTISENSORCALIBRATION_POINTCLOUD2IMAGENODELET_H
#define MULTISENSORCALIBRATION_POINTCLOUD2IMAGENODELET_H

// Std
#include <memory>
#include <string>
#include <unordered_map>

// Eigen
#include <Eigen/Geometry>

// ROS
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// multisensor_calibration
#include "../common/common.h"
#include "../common/lib3D/core/camera.hpp"
#include "../common/lib3D/core/image.hpp"

namespace multisensor_calibration
{

/**
 * @ingroup visualizer
 * @brief Class implementing the nodelet to fuse a point cloud and a camera image by projecting the
 * geometric 3D information from the point cloud into the camera image.
 */
class PointCloud2ImageNodelet : public nodelet::Nodelet
{
    //--- TYPEDEFS ---//

    typedef message_filters::sync_policies::ApproximateTime<
      InputImage_Message_T, InputCloud_Message_T>
      ImgCloudApproxSync;

    typedef message_filters::sync_policies::ExactTime<
      InputImage_Message_T, InputCloud_Message_T>
      ImgCloudExactSync;

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Default Constructor
     */
    PointCloud2ImageNodelet();

    /**
     * @brief Default Destructor
     */
    virtual ~PointCloud2ImageNodelet();

    /**
     * @brief Method to compute the camera pose for the pcl::FrustumCulling algorithm.
     *
     * @param[in] iCamera Camera object.
     * @param[out] oFrustumCullingPose 4x4 Matrix specifying camera pose
     */
    void computeCameraPoseForFrustumCulling(const lib3d::Camera& iCamera,
                                            Eigen::Matrix4f& oFrustumCullingPose) const;

    /**
     * @brief Perform frustum culling on point cloud given teh camera object.
     *
     * @param[in] ipCloud Pointer to input cloud
     * @param[in] iCamera Camera object
     * @param[in] iMinDepth Minumum depth
     * @param[in] iMaxDepth Maximum depth
     * @param[out] oCulledPointIndices List of point indices that are within the view frustum of the
     * camera.
     */
    void doFrustumCulling(const pcl::PointCloud<InputPointType>::Ptr& ipCloud,
                          const lib3d::Camera& iCamera,
                          const float& iMinDepth,
                          const float& iMaxDepth,
                          pcl::Indices& oCulledPointIndices) const;

    /**
     * This will use the point cloud and project it into the camera image where each projected point
     * is colorized based on its depth with respect to the camera.
     *
     * The colorization is performed by applying the RAINBOW colormap from
     * [`cv::ColorMap`](https://docs.opencv.org/master/d3/d50/group__imgproc__colormap.html).
     *
     * In order to reduce the size of the point cloud prior to projecting it into the camera image,
     * a frustum culling is performed.
     */
    void onSensorDataReceived(const InputImage_Message_T::ConstPtr& ipImgMsg,
                              const InputCloud_Message_T::ConstPtr ipCloudMsg);

  private:
    /**
     * @brief Method to extract image and frame id from message
     *
     * @param[in] ipImgMsg Pointer to message.
     * @param[out] oImage Image
     * @param[out] oFrameId Frame Id
     */
    inline void getImageFromMessage(const sensor_msgs::ImageConstPtr& ipImgMsg,
                                    cv::Mat& oImage,
                                    std::string& oFrameId) const;
    /**
     * @brief Method to get transform between two tf frames.
     *
     * @param[in] iTargetFrame Id of Target frame.
     * @param[in] iSourceFrame Id of Source frame.
     * @param[out] oTransform Transform from source to target frame.
     * @return True, if successful. False, otherwise.
     */
    bool getTransformBetweenTfFrames(const std::string& iTargetFrame,
                                     const std::string& iSourceFrame,
                                     tf::StampedTransform& oTransform) const;

    /**
     * @brief Method to get camera intrinsics from camera info message and store in camera_ object.
     */
    bool initializeCameraData(const std::string& iImgTopic,
                              const EImageState& iImgState,
                              const std::string& iCameraNamespace, lib3d::Camera& oCamera) const;

    /**
     * @brief Method to initialize subscribers of nodelet
     *
     * @param[in, out] ioNh Object of node handle
     * @return True, if successful. False, otherwise.
     */
    bool initializePublishers(ros::NodeHandle& ioNh);

    /**
     * @brief Method to initialize subscribers of nodelet
     *
     * @param[in, out] ioNh Object of node handle
     * @return True, if successful. False, otherwise.
     */
    bool initializeSubscribers(ros::NodeHandle& ioNh);

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
     * @brief Method to read launch parameters
     *
     * @param[in] iNh Object of node handle
     * @return True if successful. False, otherwise (e.g. if sanity check fails)
     */
    bool readLaunchParameters(const ros::NodeHandle& iNh);

    void projectCloudPointIntoDepthMap(const InputPointType& iCloudPnt,
                                       const lib3d::Camera& iCamera,
                                       lib3d::DepthMap& ioDepthMap) const;

    //--- MEMBER DECLARATION ---//

  private:
    /// name of nodelet
    std::string nodeletName_;

    /// Flag indicating initialization state of nodelet
    bool isInitialized_;

    /// Global node handler
    ros::NodeHandle nh_;

    /// message filter for approximated message synchronization for vis or nir image and point cloud
    std::shared_ptr<message_filters::Synchronizer<ImgCloudApproxSync>> pImgCloudApproxSync_;

    /// message filter for exact message synchronization for vis or nir image and point cloud
    std::shared_ptr<message_filters::Synchronizer<ImgCloudExactSync>> pImgCloudExactSync_;

    /// Subscriber to vis image topic
    image_transport::SubscriberFilter imageSubsc_;

    /// Subscriber to point cloud
    message_filters::Subscriber<InputCloud_Message_T> cloudSubsc_;

    /// Publisher for fused image
    image_transport::Publisher pub_;

    /// transform listener to get the transform between camera and point cloud
    tf::TransformListener tfListener_;

    /// State of image which is received.
    EImageState imageState_;

    /// Namespace of the camera, used to compute topic of cameraInfo
    std::string cameraNamespace_;

    /// Camera object holding intrinsics and extrinsic coordinates of the camera
    lib3d::Camera camera_;

    /// Member variable holding minimum depth value to consider when fusing image and point cloud
    float minDepth_;

    /// Member variable holding maximum depth value to consider when fusing image and point cloud
    float maxDepth_;

    /// Flag indicating to use temporary transform between sensors which is passed as launch parameter
    bool useTemporaryTransform_;

    /// Temporary transform between sensors to be used if useTemporaryTransform_ is true
    tf::Transform temporaryTransform_;

    /// Queue size for synchronization of image messages and point cloud
    int syncQueueSize_;

    /// Flag to activate exact time synchronization
    bool useExactSync_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_POINTCLOUD2IMAGENODELET_H
