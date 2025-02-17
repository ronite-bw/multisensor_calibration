# Copyright (c) 2024 - 2025 Fraunhofer IOSB and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Fraunhofer IOSB nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#!/usr/bin/env python

from dynamic_reconfigure.parameter_generator_catkin import *
import EnumDefinitions as enums

####################################################################################################
def generateParameters(ioGenerator: ParameterGenerator, icpEnum: str = None) -> ParameterGenerator:
    """Function to generate dynamic configuration parameters for the detection of the calibration
    target in the LiDAR point cloud..

    Parameters
    ----------
    ioGenerator : ParameterGenerator, required
        Object of class ParameterGenerator to which the parameters are added.

    Returns
    ------
        Updated copy of ioGenerator
    """
    
    # target_detection_group
    target_detection_group: ParameterGenerator.Group 
    target_detection_group = ioGenerator.add_group(name="LiDAR Target Detection", type="tab")

    # normal_estimation_group
    normal_estimation_group: ParameterGenerator.Group 
    normal_estimation_group = target_detection_group.add_group(name="Normal Estimation")
    normal_estimation_group.add("search_method", int_t, 0, 
                                "Select method to use for neighbor search.", 
                                1, edit_method=enums.searchMethod(ioGenerator))
    normal_estimation_group.add("search_radius", double_t, 0, 
                                "Radius in which to search for neighbors. In case of "
                                "'RadiusSearch', this is a spatial extend. In case of "
                                "'NearestNeighborSearch', this represents the number of nearest "
                                "neighbors (truncated to int).", 
                                100.0, 0.001, 500.0)

    # region_growing_group
    region_growing_group: ParameterGenerator.Group 
    region_growing_group = target_detection_group.add_group(name="Region Growing")
    region_growing_group.add("cluster_size_min", int_t, 0, 
            "Minimum number of points a cluster needs to contain in order to be considered as "
            "valid inside the region growing.", 100, 10, 999)
    region_growing_group.add("cluster_size_max", int_t, 0, 
            "Maximum number of points a cluster needs to contain in order to be considered as "
            "valid inside the region growing.", 10000, 1000, 999999)
    region_growing_group.add("number_neighbors", int_t, 0, 
            "Number of neighbor points to consider during region growing.", 30, 1, 100)
    region_growing_group.add("angle_thresh", double_t, 0, 
            "Angle in degrees used as the allowable range for the normals deviation. "
            "If the deviation between points normals is less than the smoothness threshold "
            "then they are suggested to be in the same cluster", 1.8, 0.1, 10.0)
    region_growing_group.add("curvature_thresh", double_t, 0, 
            "Second criteria for the region growing.  If two points have a small normals deviation "
            "then the disparity between their curvatures is tested.", 0.8, 0.1, 10.0)
    
    # size_filter_group
    size_filter_group: ParameterGenerator.Group
    size_filter_group = target_detection_group.add_group(name="Size Filter")
    size_filter_group.add("width_min_tolerance", double_t, 0, 
            "Tolerance (in m) of the minimum board width when filtering the clusters based on "
            "their size.", 0.05, 0.01, 0.5)
    size_filter_group.add("width_max_tolerance", double_t, 0, 
            "Tolerance (in m) of the maximum board width when filtering the clusters based on "
            "their size.", 0.1, 0.01, 0.5)
    size_filter_group.add("height_min_tolerance", double_t, 0, 
            "Tolerance (in m) of the minimum board height when filtering the clusters based on "
            "their size.", 0.05, 0.01, 0.5)
    size_filter_group.add("height_max_tolerance", double_t, 0, 
            "Tolerance (in m) of the maximum board height when filtering the clusters based on "
            "their size.", 0.5, 0.01, 1.0)
    
    # ransac_group
    ransac_group: ParameterGenerator.Group
    ransac_group = target_detection_group.add_group(name="RANSAC")
    ransac_group.add("distance_thresh", double_t, 0, 
            "Distance threshold (in m) from model for points to count as inliers during RANSAC.", 
            0.05, 0.01, 0.5)
    ransac_group.add("rotation_variance", double_t, 0, 
            "Maximum angle in rotation (in degrees) to be sampled when computing the new "
            "coefficients within RANSAC.", 
            1.0, 0.01, 180.0)
    ransac_group.add("translation_variance", double_t, 0, 
            "Maximum distance in translation (in m) to be sampled when computing the new "
            "coefficients within RANSAC.", 
            0.08, 0.01, 1.0)
    
    if icpEnum is None:
       icpEnum = enums.icpMethod(ioGenerator)

    # ransac_optimize_group
    ransac_optimize_group: ParameterGenerator.Group
    ransac_optimize_group = ransac_group.add_group(name="optimize_coefficients", type="collapse",
                                                   state=True);
    ransac_optimize_group.add("target_icp_variant", int_t, 0,
                              "Select ICP variant to use to optimize coefficients.", 
                              2, edit_method=icpEnum)
    ransac_optimize_group.add("target_icp_max_correspondence_distance", double_t, 0,
                              "Maximum distance for ICP to search for point correspondences. "
                              "Given as ratio with respect to shorter side of calibration target.", 
                              0.1, 0.001, 1.0)
    ransac_optimize_group.add("target_icp_rotation_tolerance", double_t, 0,
                              "Rotation tolerance for convergence check. Given in degrees.", 
                              0.5, 0.001, 10.0)
    ransac_optimize_group.add("target_icp_translation_tolerance", double_t, 0,
                              "Translation tolerance for convergence check. Given in unit of the"
                              "LiDAR point cloud, typically meters.", 
                              0.001, 0.000001, 1.0)

    
    return ioGenerator