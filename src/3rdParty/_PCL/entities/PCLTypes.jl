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


"""
- `_PCL_ENDIAN_BIG_BYTE`, byte-swapped big-endian.
- `_PCL_ENDIAN_BIG_WORD`, word-swapped big-endian.
- `_PCL_ENDIAN_LITTLE_BYTE`, byte-swapped little-endian.
- `_PCL_ENDIAN_LITTLE_WORD`, word-swapped little-endian.

Notes:
- https://docs.julialang.org/en/v1/base/io-network/#Base.ENDIAN_BOM
- https://www.boost.org/doc/libs/1_69_0/boost/predef/other/endian.h

DevNotes
- FIXME, `_PCL_ENDIAN_BIG_WORD`, `_PCL_ENDIAN_LITTLE_BYTE` -- values are not correct
"""
@enum _PCL_ENDIAN begin
  _PCL_ENDIAN_BIG_BYTE=0x01020304 
  _PCL_ENDIAN_BIG_WORD=0x00020304 
  _PCL_ENDIAN_LITTLE_BYTE=0x04030200 
  _PCL_ENDIAN_LITTLE_WORD=0x04030201
end

abstract type PointT end

"""
    $TYPEDEF

Immutable PointXYZ with color information.  E.g. `PointXYZ{RGB}`, `PointXYZ{Gray}`, etc.

Aliases
- `PointXYZRGB`
- `PointXYZRGBA`

See 
- https://pointclouds.org/documentation/structpcl_1_1_point_x_y_z.html
- https://pointclouds.org/documentation/point__types_8hpp_source.html
"""
Base.@kwdef struct PointXYZ{C <: Colorant, T <: Number} <: PointT
  color::C           = RGBA(1,1,1,1)
  data::SVector{4,T} = SVector(0,0,0,Float32(1))
end


"""
    $TYPEDEF

Immutable Header.

See https://pointclouds.org/documentation/structpcl_1_1_p_c_l_header.html
"""
Base.@kwdef struct Header
  seq::UInt32      = UInt32(0)
  stamp::UInt64    = UInt64(0)
  frame_id::String = ""
end

Base.@kwdef struct PointField
  name::String
  offset::UInt32   = UInt32(0)
  datatype::UInt8  = UInt8(0)
  count::UInt32    = UInt32(0)
end

"""
    $TYPEDEF

Immutable point cloud type. Immutable for performance, computations are more 
frequent and intensive than anticipated frequency of constructing new clouds.

References:
- https://pointclouds.org/documentation/structpcl_1_1_p_c_l_point_cloud2.html
- https://pointclouds.org/documentation/classpcl_1_1_point_cloud.html
- https://pointclouds.org/documentation/common_2include_2pcl_2point__cloud_8h_source.html
"""
Base.@kwdef struct PointCloud{T<:PointT}
  """ the point cloud header """
  header::Header           = Header()
  """ the point cloud height (if organized as image structure).  Specifies the height 
  of the point cloud dataset in the number of points. HEIGHT has two meanings:
  - it can specify the height (total number of rows) of an organized point cloud dataset;
  - it is set to 1 for unorganized datasets (thus used to check whether a dataset is organized or not). """
  height::UInt32           = UInt32(0)
  """ the point cloud width (if organized as image structure).  Specifies the width 
  of the point cloud dataset in the number of points. WIDTH has two meanings:
  - it can specify the total number of points in the cloud (equal with POINTS see below) for unorganized datasets;
  - it can specify the width (total number of points in a row) of an organized point cloud dataset. """
  width::UInt32            = UInt32(0)
  """ `Vector` of `<:PointT` representing the point cloud """
  fields::Vector{PointField}= Vector{PointField}()
  data::Vector{T}          = Vector{PointXYZ{RGB{Colors.FixedPointNumbers.N0f8}, Float32}}()
  """ WARNING, untested """
  is_bigendian::_PCL_ENDIAN= (en=instances(_PCL_ENDIAN); en[findfirst(Int.(en) .== Int.(Base.ENDIAN_BOM))])
  point_step::UInt32       = UInt32(0)
  row_step::UInt32         = UInt32(0)
  """ true if points are invalid (e.g., have NaN or Inf values in any of their floating point fields). """
  is_dense::UInt8          = UInt8(0)
  # """ Sensor acquisition pose (origin/translation), optional."""
  # sensor_origin_::P        = SA[0;0;0.0]
  # """ sensor acquisition pose (rotation), optional."""
  # sensor_orientation_::R   = SMatrix{3,3}(1.0,0,0,0,1,0,0,0,1)
end
