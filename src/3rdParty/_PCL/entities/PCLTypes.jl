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
    _PCL_ENDIAN

FIXME These values have not been validated!

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

Base.convert(::Type{<:_PCL_ENDIAN}, val::Integer) = (en=instances(_PCL_ENDIAN); en[findfirst(Int.(en) .== Int.(convert(UInt32, val)))])

"""
Format`:::UInt8` as enum for PointCloud2 messages between ROS and PCL.

References
- https://docs.ros.org/en/api/sensor_msgs/html/msg/PointField.html
"""
@enum _PCL_POINTFIELD_FORMAT begin
  _PCL_INT8    = UInt8(1)
  _PCL_UINT8   = UInt8(2)
  _PCL_INT16   = UInt8(3)
  _PCL_UINT16  = UInt8(4)
  _PCL_INT32   = UInt8(5)
  _PCL_UINT32  = UInt8(6)
  _PCL_FLOAT32 = UInt8(7)
  _PCL_FLOAT64 = UInt8(8)
end

Base.convert(::Type{<:_PCL_POINTFIELD_FORMAT}, val::Integer) = (en=instances(_PCL_POINTFIELD_FORMAT); en[findfirst(Int.(en) .== Int.(convert(UInt8, val)))])

struct asType{T} end

(::Type{<:asType{_PCL_INT8}})()    = Int8
(::Type{<:asType{_PCL_UINT8}})()   = UInt8
(::Type{<:asType{_PCL_INT16}})()   = Int16
(::Type{<:asType{_PCL_UINT16}})()  = UInt16
(::Type{<:asType{_PCL_INT32}})()   = Int32
(::Type{<:asType{_PCL_UINT32}})()  = UInt32
(::Type{<:asType{_PCL_FLOAT32}})() = Float32
(::Type{<:asType{_PCL_FLOAT64}})() = Float64


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
  data::SVector{4,T} = SVector(0,0,0,1f0)
end


"""
    $TYPEDEF

Immutable Header.

See https://pointclouds.org/documentation/structpcl_1_1_p_c_l_header.html
"""
Base.@kwdef struct Header
  """ The sequence number """
  seq::UInt32      = UInt32(0)
  """ A timestamp associated with the time when the data was acquired. 
  The value represents microseconds since 1970-01-01 00:00:00 (the UNIX epoch). 
  Suggest: making this relative to UCT to account for timezones in future. """
  stamp::UInt64    = UInt64(0)
  """ Coordinate frame ID. """
  frame_id::String = ""
end

# https://pointclouds.org/documentation/structpcl_1_1_p_c_l_point_field.html
Base.@kwdef struct PointField
  """ name of field """
  name::String
  """ Offset from start of point struct """
  offset::UInt32                   = UInt32(0)
  """ Datatype enumeration, see _PCL_POINTFIELD_FORMAT """
  datatype::_PCL_POINTFIELD_FORMAT = convert(_PCL_POINTFIELD_FORMAT, 7)
  """ How many elements in the field """
  count::UInt32                    = UInt32(0)
end

# https://docs.ros.org/en/hydro/api/pcl/html/structpcl_1_1detail_1_1FieldMapping.html
# https://pointclouds.org/documentation/common_2include_2pcl_2point__cloud_8h_source.html#l00072
Base.@kwdef mutable struct FieldMapping
  serialized_offset::UInt32  = UInt32(0)
  struct_offset::UInt32      = UInt32(0)
  size::UInt32               = UInt32(0)
end

struct FieldMatches{T<:PointT} end

# https://docs.ros.org/en/hydro/api/pcl/html/structpcl_1_1detail_1_1FieldAdder.html
Base.@kwdef struct FieldAdder
  fields_::Vector{<:PointField} = Vector{PointField}()
end

# https://docs.ros.org/en/hydro/api/pcl/html/point__cloud_8h_source.html#l00066
# https://pointclouds.org/documentation/common_2include_2pcl_2point__cloud_8h_source.html#l00072
const MsgFieldMap = Vector{FieldMapping}

# https://docs.ros.org/en/hydro/api/pcl/html/conversions_8h_source.html#l00091
Base.@kwdef struct FieldMapper{T<:PointT}
  fields_::Vector{<:PointField} = Vector{PointField}()
  map_::Vector{<:FieldMapping}  = Vector{FieldMapping}()
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
Base.@kwdef struct PCLPointCloud2
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
  """ field descriptions of data """
  fields::Vector{<:PointField}= Vector{PointField}()
  """ `Vector` of `<:PointT` representing the point cloud """
  data::Vector{UInt8}      = Vector{UInt8}()
  """ WARNING, untested """
  is_bigendian::_PCL_ENDIAN= convert(_PCL_ENDIAN, Base.ENDIAN_BOM)
  point_step::UInt32       = UInt32(0)
  row_step::UInt32         = UInt32(0)
  """ true if points are invalid (e.g., have NaN or Inf values in any of their floating point fields). """
  is_dense::UInt8          = UInt8(0)
end


"""
    $TYPEDEF

Convert a PCLPointCloud2 binary data blob into a `Caesar._PCL.PointCloud{T}` object using 
a `field_map::Caesar._PCL.MsgFieldMap`.

Use `PointCloud(::Caesar._PCL.PCLPointCloud2)` directly or create you
own `MsgFieldMap`:

```julia
field_map = Caesar._PCL.createMapping(msg.fields, field_map)
```

Notes
- Tested on Radar data with height `z=constant` for all points -- i.e. 2D sweeping scan where `.height=1`.

DevNotes
- TODO .PCLPointCloud2 convert not tested on regular 3D data from structured light or lidar yet, but current implementation should be close (or already working).


References
- https://pointclouds.org/documentation/classpcl_1_1_point_cloud.html
- (seems older) https://docs.ros.org/en/hydro/api/pcl/html/conversions_8h_source.html#l00123 
"""
Base.@kwdef struct PointCloud{T<:PointT,P,R}
  """ the point cloud header """
  header::Header            = Header()
  """ `Vector` of `UInt8` representing the cloud points """
  points::Vector{T}         = Vector{typeof(PointXYZ())}() # {PointXYZ{RGB{Colors.FixedPointNumbers.N0f8}, Float32}}
  """ the point cloud width (if organized as image structure).  Specifies the width 
  of the point cloud dataset in the number of points. WIDTH has two meanings:
  - it can specify the total number of points in the cloud (equal with POINTS see below) for unorganized datasets;
  - it can specify the width (total number of points in a row) of an organized point cloud dataset. """
  width::UInt32            = UInt32(0)
  """ the point cloud height (if organized as image structure).  Specifies the height 
  of the point cloud dataset in the number of points. HEIGHT has two meanings:
  - it can specify the height (total number of rows) of an organized point cloud dataset;
  - it is set to 1 for unorganized datasets (thus used to check whether a dataset is organized or not). """
  height::UInt32           = UInt32(0)
  """ true if points are invalid (e.g., have NaN or Inf values in any of their floating point fields). """
  is_dense::Bool           = false
  """ Sensor acquisition pose (origin/translation), optional."""
  sensor_origin_::P        = SA[0f0;0;0]
  """ sensor acquisition pose (rotation), optional."""
  sensor_orientation_::R   = SMatrix{3,3}(1f0,0,0,0,1,0,0,0,1)
end


#