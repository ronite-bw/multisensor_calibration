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

####################################################################################################
def generateParameters(ioGenerator: ParameterGenerator) -> ParameterGenerator:
    """Function to generate dynamic configuration parameters for PnP algorithm to calibrate camera
    and LiDAR.

    Parameters
    ----------
    ioGenerator : ParameterGenerator, required
        Object of class ParameterGenerator to which the parameters are added.

    Returns
    ------
        Updated copy of ioGenerator
    """

    # Perspective-n-Point
    pnp_group: ParameterGenerator.Group
    pnp_group = ioGenerator.add_group("PnP Registration", type="tab")

    pnp_group.add("pnp_limit_board_rpj_error", bool_t, 0,
            "Use max maximum reprojection error to accept during calibration of a single target "
            "pose. If false, 'board_max_rpj_error' is ignored.",
            False)

    pnp_group.add("pnp_board_max_rpj_error", double_t, 0, 
            "Limit for maximum reprojection error to accept during calibration of a single target "
            "pose. All calibrated poses, that exceed this limit are rejected.", 5.0, 0.0, 5.0)
    
    pnp_group.add("pnp_board_min_inliers", int_t, 0, 
            "Threshold for minimum number of inliers to accept during calibration of a single "
            "target pose. All calibrated poses, that do not reach this threshold are rejected.", 
            10, 4, 16)

    pnp_group.add("pnp_inlier_max_rpj_error", double_t, 0, 
            "Limit for maximum reprojection error for which points are considered as RANSAC "
            "inliers during solvePnP.", 8.0, 0.0, 10.0)
    
    return ioGenerator