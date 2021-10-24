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

using DocStringExtensions
using StaticArrays
using ..Colors

## hold off on exports, users can in the mean-time use/import via e.g. _PCL.PointXYZ
# export PointT, PointXYZ, PointXYZRGB, PointXYZRGBA
# export PCLHeader, PointCloud

abstract type PointT end

"""
    $TYPEDEF

See https://pointclouds.org/documentation/point__types_8hpp_source.html
"""
Base.@kwdef struct PointXYZ{C <: Colorant, T <: Number} <: PointT
  color::C           = RGBA(1,1,1,1)
  data::SVector{4,T} = SVector(0,0,0,Float32(1))
end

# Construct helpers nearest to PCL
PointXYZRGBA( x::Real=0, y::Real=0, z::Real=Float32(1); 
              r::Real=1,g::Real=1,b::Real=1, alpha::Real=1,
              color::Colorant=RGBA(r,g,b,alpha), 
              pos=SA[x,y,z], data=SA[pos...,1] ) = PointXYZ(;color,data)
#
PointXYZRGB(  x::Real=0, y::Real=0, z::Real=Float32(1); 
              r::Real=1,g::Real=1,b::Real=1, 
              color::Colorant=RGB(r,g,b), 
              pos=SA[x,y,z], data=SA[pos...,1] ) = PointXYZ(;color,data)
#

function Base.getproperty(p::PointXYZ, f::Symbol)
  if f == :x 
    return getfield(p, :data)[1]
  elseif f == :y 
    return getfield(p, :data)[2]
  elseif f == :z
    return getfield(p, :data)[3]
  elseif f == :r
    return getfield(p, :color).r
  elseif f == :g
    return getfield(p, :color).g
  elseif f == :b
    return getfield(p, :color).b
  elseif f == :alpha
    return getfield(p, :color).alpha
  elseif f == :data || f == :pos
    return getfield(p, :data)
  elseif f == :color || f == :rgb
    return getfield(p, :color)
  end
  error("PointXYZ has no field $f")
end


"""
    $TYPEDEF

See https://pointclouds.org/documentation/structpcl_1_1_p_c_l_header.html
"""
Base.@kwdef struct PCLHeader
  seq::UInt32      = UInt32(0)
  stamp::UInt64    = UInt64(0)
  frame_id::String = ""
end

Base.@kwdef struct PointCloud
  header::PCLHeader        = PCLHeader()
  """ vector of points representing the point cloud """
  points::Vector{<:PointT} = Vector{PointXYZ{RGBA{Colors.FixedPointNumbers.N0f8}, Float32}}()
end



end