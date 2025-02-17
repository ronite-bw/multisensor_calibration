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
def searchMethod(gen: ParameterGenerator) -> str:
    """Get enum representing different search methods within a point cloud"""
    
    return gen.enum([gen.const(name="RadiusSearch", 
                               type=int_t, 
                               value=0,
                               descr="Search for neighbors within a given radius."),
                     gen.const(name="NearestNeighborSearch",
                               type=int_t,
                               value=1,
                               descr="Search for given number of neighbors.")],
                      "Available methods for neighbor search.")

####################################################################################################
def icpMethod(gen: ParameterGenerator) -> str:
    """Get enum representing different variants of ICP"""
    
    return gen.enum([gen.const(name="ICP", 
                               type=int_t, 
                               value=0,
                               descr="Standard Point-to-Point ICP."),
                     gen.const(name="PlaneICP",
                               type=int_t,
                               value=1,
                               descr="Point-to-Plane ICP."),
                     gen.const(name="GICP",
                               type=int_t,
                               value=2,
                               descr="Generalized ICP.")],
                      "Available ICP variants.")