# NEWS Caesar.jl

Major changes and news in Caesar.jl.

## v0.15 - v0.16

- Change to standard weakdeps package extensions, dropping use of Requires.jl.
- Refactor and separate out to new PyCaesar.jl package all code using PyCall.
- Updates for Julia 1.10.
- Updates for IncrementalInference upgrades relating to StaticArray variable values.
- Manifold updates to factors.
- Downstreamed std ROS handlers to PyCaesar.
- Fix `saveLAS` to use `Int32`.
- Several compat updates for dependencies.
- Restore Docs build, and update links for NavAbility at WhereWhen.ai Technologies Inc.
- Introduce `FolderDict` as data structure for lower memory consumption, also as potential BlobStore.

## Changes in v0.13

- Finally adding a NEWS.md to Caesar.jl.
- Standardize all Manifolds.jl representations to use `SciML/ArrayPartition` instead of `Manifold/ProductRepr`.
- Add `RobotOS / PyCall` based interface for writing bagfiles with `RosbagWriter`.
- Add `_PCL` export functions to go from `Caesar._PCL.PointCloud` out to `PCLPointCloud2` and `ROS.PointCloud2` types.
- Refactored `ScatterAlign` to support both `ScatterAlignPose2` and `ScatterAlignPose3`.
- Improved `_PCL.apply` to now transform both 2D and 3D pointclouds using `Manifolds.SpecialEuclidean(2)` and `Manifolds.SpecialEuclidean(3)`.
- Added more testing on `_PCL` conversions and transforms.
- Added `ICP` for point cloud alignment, (#885, #886).
- Separated test data to `JuliaRobotics/CaesarTestData.jl` (#885).
