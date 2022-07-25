# Pointclouds and PCL Types

## Introduction Caesar._PCL

A wide ranging and well used [point cloud library exists called PCL](https://pointclouds.org/) which is implemented in C++.  To get access to many of those features and bridge the Caesar.jl suite of packages, the base `PCL.PointCloud` types have been implemented in Julia and reside under `Caesar._PCL`.  The main types of interest:
- [`Caesar._PCL.PointCloud`](@ref),
- [`Caesar._PCL.PCLPointCloud2`](@ref),
- [`Caesar._PCL.PointXYZ`](@ref),
- [`Caesar._PCL.Header`](@ref),
- [`Caesar._PCL.PointField`](@ref),
- [`Caesar._PCL.FieldMapper`](@ref).

These types are conditionally loaded once `Colors.jl` is included (see [Requires.jl](https://github.com/JuliaPackaging/Requires.jl)),
```julia
using Colors, Caesar
using StaticArrays

# one point
x,y,z,intens = 1f0,0,0,1
pt = Caesar._PCL.PointXYZ(;data=SA[x,y,z,intens])

# etc.
```

```@docs
Caesar._PCL.PointCloud
```

## Conversion with `ROS.PointCloud2`

Strong integration between PCL and [ROS](http://www.ros.org) predominantly through the message types
- `@rosimport std_msgs.msg: Header`, `@rosimport sensor_msgs.msg: PointField`, `@rosimport sensor_msgs.msg: PointCloud2`.

These have been integrated through conversions to equivalent Julian types already listed above.  ROS conversions requires RobotOS.jl be loaded, see page on using [ROS Direct](@ref ros_direct).

```@docs
Caesar._PCL.PCLPointCloud2
Caesar._PCL.PointXYZ
Caesar._PCL.Header
Caesar._PCL.PointField
Caesar._PCL.FieldMapper
```

## Aligning Point Clouds

Caesar.jl is currently growing support for two related point cloud alignment methods, namely:
- Continuous density function alignment [`ScatterAlignPose2`](@ref), [`ScatterAlignPose3`](@ref),
- Traditional Iterated Closest Point (with normals) [`alignICP_Simple`](@ref).

### [`ScatterAlign` for `Pose2` and `Pose3`](@id sec_scatter_align)

These factors use minimum mean distance embeddings to cost the alignment between pointclouds and supports various other interesting function alignment cases.  These functions require `Images.jl`, see page [Using Images](@ref images_and_fiducials).

```@docs
Caesar.ScatterAlign
Caesar.ScatterAlignPose2
Caesar.ScatterAlignPose3
```

!!! note
    Future work may include `ScatterAlignPose2z`, please open issues at Caesar.jl if this is of interest.

### Iterative Closest Point

Ongoing work is integrating ICP into a factor similar to `ScatterAlign`.

```@docs
Caesar._PCL.alignICP_Simple
```

## Visualizing Point Clouds

See work in progress on alng with example code on the page [3D Visualization](@ref viz_pointcloud_makie).
