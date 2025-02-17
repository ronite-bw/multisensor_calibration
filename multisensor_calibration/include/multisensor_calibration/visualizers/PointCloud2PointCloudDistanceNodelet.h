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

#ifndef MUTLISENSORCALIBRATION_POINTCLOUD2POINTCLOUDDISTANCENODELET_H
#define MUTLISENSORCALIBRATION_POINTCLOUD2POINTCLOUDDISTANCENODELET_H

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif

// Std
#include <functional>
#include <memory>
#include <mutex>
#include <string>

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// multisensor_calibration
#include "../common/common.h"

namespace multisensor_calibration
{

/**
 * @ingroup visualizer
 * @brief Class implementing the nodelet to calculate the distance of a source point cloud to a
 * target point cloud.
 */
class PointCloud2PointCloudDistanceNodelet : public nodelet::Nodelet
{
    //--- TYPEDEFS ---//

  private:
    /// Typedef for point cloud message to use
    typedef sensor_msgs::PointCloud2 PointCloud;

    //--- ENUMS ---//

  public:
    /// Enumeration of distance measures to be used for calculation
    enum EDistanceMeasure
    {
        POINT_2_POINT = 0,
        POINT_2_SURFACE
    };

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Default Constructor
     */
    PointCloud2PointCloudDistanceNodelet();

    /**
     * @brief Default Destructor
     */
    virtual ~PointCloud2PointCloudDistanceNodelet();

  private:
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
     * @brief Message callback receiving point clouds. This stores the received point cloud as well
     * as the message header in the corresponding list at the given index position.
     *
     * @param[in] ipCloudMsg Pointer to message holding point cloud.
     * @param[in] idx Index position.
     */
    void cloudMessageCallback(const InputCloud_Message_T::ConstPtr& ipCloudMsg, const int idx);

    /**
     * @brief Method to process the received point cloud. This is triggered by the processing timer
     * at a specified processing rate.
     *
     * @param[in] e TimerEvent. Currently unused.
     */
    void processCloudsCallback(const ros::TimerEvent& e);

    /**
     * @brief Calculate point-to-point distance between #iSourcePnt and the nearest neighbors in
     * #iTargetCloud.
     *
     * This will calculate the euclidean distance to the closest of the nearest neighbors referenced
     * by #iNnIndices.
     *
     * @param[in] iSourcePnt Source point for which to calculate the distance.
     * @param[in] iTargetCloud Target cloud from which the nearest neighbors are fount and to
     * which the distance is to be calculated.
     * @param[in] iTargetNormals Normal vectors in the target cloud, representing the surface normal
     * for each point in #iTargetCloud.
     * @param[in] iNnIndices Indices of the nearest neighbors of #iSourcePnt in #iTargetCloud.
     * @param[in] iNnDistances Squared distances of the nearest neighbors to #iSourcePnt.
     * @return Distance
     */
    static float calculatePoint2PointDistance(const InputPointType& iSourcePnt,
                                              const pcl::PointCloud<InputPointType>::Ptr& iTargetCloud,
                                              const pcl::PointCloud<pcl::Normal>::Ptr& iTargetNormals,
                                              const pcl::IndicesPtr& iNnIndices,
                                              const std::vector<float>& iNnDistances);

    /**
     * @brief Calculate point-to-surface distance between #iSourcePnt and the nearest neighbors in
     * #iTargetCloud.
     *
     * This will loop through all neighboring points referenced by #iNnIndices and, together with
     * the surface normals in #iTargetNormals, parameterize a corresponding surface plane for each
     * neighbor point. Then, it will calculate the orthogonal distance of #iSourcePnt from each of
     * these planes and return the smallest, i.e. the winner-takes-it-all solution.
     *
     * @param[in] iSourcePnt Source point for which to calculate the distance.
     * @param[in] iTargetCloud Target cloud from which the nearest neighbors are fount and to
     * which the distance is to be calculated.
     * @param[in] iTargetNormals Normal vectors in the target cloud, representing the surface normal
     * for each point in #iTargetCloud.
     * @param[in] iNnIndices Indices of the nearest neighbors of #iSourcePnt in #iTargetCloud.
     * @param[in] iNnDistances Squared distances of the nearest neighbors to #iSourcePnt.
     * @return Distance
     */
    static float calculatePoint2SurfaceDistance(const InputPointType& iSourcePnt,
                                                const pcl::PointCloud<InputPointType>::Ptr& iTargetCloud,
                                                const pcl::PointCloud<pcl::Normal>::Ptr& iTargetNormals,
                                                const pcl::IndicesPtr& iNnIndices,
                                                const std::vector<float>& iNnDistances);

    //--- MEMBER DECLARATION ---//

  private:
    /// Global node handler
    ros::NodeHandle nh_;

    /// List of subscribers for the point cloud
    std::vector<ros::Subscriber> cloudSubscrs_;

    /// Publisher for enhanced source point cloud
    std::vector<ros::Publisher> pubs_;

    /// Transform listener to look up transform between both input clouds
    tf::TransformListener tfListener_;

    /// Pointers to received point clouds
    std::vector<pcl::PointCloud<InputPointType>::Ptr> pPointClouds_;

    /// List of message headers of received point clouds
    std::vector<std_msgs::Header> pointCloudMsgHeaders_;

    /// Mutex to control access to point clouds and the corresponding meta data
    std::mutex pointCloudMutex_;

    /// Timer to trigger cloud processing
    ros::Timer processingTimer_;

    /// Number of clouds to process. Default: 2
    int nClouds_;

    /// Distance measure to use
    EDistanceMeasure distanceMeasure_;

    /// Number of nearest neighbors to consider in the target cloud for each point in the source
    /// cloud when calculating the distance. Default: 5.
    int nNearestNeighbors_;

    /// Maximum distance at which to truncate the calculated distance.
    /// This is also used for normalization when calculating the intensity. Default: 5.0;
    float maxDistance_;

    /// Distance at which to clamp the source point cloud. Point exceeding the distance are removed.
    /// Default: FLT_MAX
    float clampDistanceThreshold_;

    /// Function pointer to the function used to calculate the distance
    std::function<float(const InputPointType&, const pcl::PointCloud<InputPointType>::Ptr&,
                        const pcl::PointCloud<pcl::Normal>::Ptr&,
                        const pcl::IndicesPtr&, const std::vector<float>&)>
      calculateDistanceFn_;

    /// Flag indicating to use temporary transform between sensors which is passed as launch parameter
    bool useTemporaryTransform_;

    /// Temporary transform between sensors to be used if useTemporaryTransform_ is true
    tf::Transform temporaryTransform_;
};

} // namespace multisensor_calibration

#endif // MUTLISENSORCALIBRATION_POINTCLOUD2POINTCLOUDDISTANCENODELET_H
