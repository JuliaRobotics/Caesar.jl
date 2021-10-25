

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

Immutable PCLHeader.

See https://pointclouds.org/documentation/structpcl_1_1_p_c_l_header.html
"""
Base.@kwdef struct PCLHeader
  seq::UInt32      = UInt32(0)
  stamp::UInt64    = UInt64(0)
  frame_id::String = ""
end

"""
    $TYPEDEF

Immutable point cloud type. Immutable for performance, computations are more 
frequent and intensive than anticipated frequency of constructing new clouds.

References:
- https://pointclouds.org/documentation/classpcl_1_1_point_cloud.html
- https://pointclouds.org/documentation/common_2include_2pcl_2point__cloud_8h_source.html
"""
Base.@kwdef struct PointCloud{T<:PointT,P,R}
  """ the point cloud header """
  header::PCLHeader        = PCLHeader()
  """ `Vector` of `<:PointT` representing the point cloud """
  data::Vector{T} = Vector{PointXYZ{RGB{Colors.FixedPointNumbers.N0f8}, Float32}}()
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
  is_dense::Bool           = true
  """ Sensor acquisition pose (origin/translation), optional."""
  sensor_origin_::P        = SA[0;0;0.0]
  """ sensor acquisition pose (rotation), optional."""
  sensor_orientation_::R   = SMatrix{3,3}(1.0,0,0,0,1,0,0,0,1)
end
