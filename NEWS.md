# NEWS Caesar.jl

Major changes and news in Caesar.jl.

## Changes in v0.13

- Finally adding a NEWS.md to Caesar.jl.
- Standardize all Manifolds.jl representations to use `SciML/ArrayPartition` instead of `Manifold/ProductRepr`.
- Add `RobotOS / PyCall` based interface for writing bagfiles with `RosbagWriter`.
- Add `_PCL` export functions to go from `Caesar._PCL.PointCloud` out to `PCLPointCloud2` and `ROS.PointCloud2` types.
- Refactored `ScatterAlign` to support both `ScatterAlignPose2` and `ScatterAlignPose3`.
- Improved `_PCL.apply` to now transform both 2D and 3D pointclouds using `Manifolds.SpecialEuclidean(2)` and `Manifolds.SpecialEuclidean(3)`.
- Added more testing on `_PCL` conversions and transforms.
