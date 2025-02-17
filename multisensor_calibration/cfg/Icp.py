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
    """Function to generate dynamic configuration parameters for ICP algorithm to calibrate
    multiple LiDAR sensors.

    Parameters
    ----------
    ioGenerator : ParameterGenerator, required
        Object of class ParameterGenerator to which the parameters are added.

    Returns
    ------
        Updated copy of ioGenerator
    """

    if icpEnum is None:
       icpEnum = enums.icpMethod(ioGenerator)

    # Iterative Closest Point
    icp_group: ParameterGenerator.Group
    icp_group = ioGenerator.add_group("ICP Registration", type="tab")

    icp_group.add("registration_icp_variant", int_t, 0,
                  "Select ICP variant to use.", 
                  2, edit_method=icpEnum)
    icp_group.add("registration_icp_max_correspondence_distance", double_t, 0,
                  "Maximum distance for ICP to search for point correspondences. "
                  "Given in unit of the LiDAR point cloud, usually meters.", 
                  0.1, 0.001, 10.0)
    icp_group.add("registration_icp_rotation_tolerance", double_t, 0,
                  "Rotation tolerance for convergence check. Given in degrees.", 
                  0.5, 0.001, 10.0)
    icp_group.add("registration_icp_translation_tolerance", double_t, 0,
                  "Translation tolerance for convergence check. Given in unit of the"
                  "LiDAR point cloud, usually meters.", 
                              0.001, 0.000001, 1.0)
    
    return ioGenerator