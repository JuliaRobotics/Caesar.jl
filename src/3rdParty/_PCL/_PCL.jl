"""
  Software License Agreement (BSD License)

  Point Cloud Library (PCL) - www.pointclouds.org
  Copyright (c) 2010, Willow Garage, Inc.
  Copyright (c) 2012-, Open Perception, Inc.

  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holder(s) nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
"""

@info "Caesar.jl is loading (but not exporting) tools requiring Colors.jl; e.g. Caesar._PCL"

module _PCL

using ..Colors

using DocStringExtensions
using StaticArrays
using Requires
import Rotations as _Rot

# Going to add dispatches on these functions
import Base: getindex, setindex!, resize!, cat

## hold off on exports, users can in the mean-time use/import via e.g. _PCL.PointXYZ
# export PointT, PointXYZ, PointXYZRGB, PointXYZRGBA
# export PCLHeader, PointCloud


# bring in the types
include("entities/PCLTypes.jl")
# bring in further source code
include("services/PointCloud.jl")


function __init__()
  @require RobotOS="22415677-39a4-5241-a37a-00beabbbdae8" include("services/ROSConversions.jl")
end


end