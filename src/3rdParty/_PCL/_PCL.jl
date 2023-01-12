
# @info "Caesar.jl is loading (but not exporting) tools requiring Colors.jl; e.g. Caesar._PCL"

module _PCL

# using ..Colors
import ..Caesar: _FastTransform3D
import ..Caesar: homogeneous_coord_to_euler_coord, euler_coord_to_homogeneous_coord, euler_angles_to_linearized_rotation_matrix
import ..Caesar: _SE3_MANI # create_homogeneous_transformation_matrix
import ..Caesar: getBelief, mean

using FileIO
using Colors
using Dates
using Printf
using DocStringExtensions
using Requires
using StaticArrays
using Statistics
using StatsBase
using LinearAlgebra
using NearestNeighbors
using Manifolds
import Rotations as _Rot
import GeometryBasics as GeoB # name collisions on members: Point, etc.
using DistributedFactorGraphs
using TensorCast
using UUIDs
using MultivariateStats

# FIXME REMOVE, only used for legacy getDataPointCloud
using Serialization 

# Going to add dispatches on these functions
import Base: getindex, setindex!, resize!, cat, convert, sizeof, hasproperty, getproperty

# gets overloaded
import Manifolds: apply
import IncrementalInference: ArrayPartition

## hold off on exports, users can in the mean-time use/import via e.g. _PCL.PointXYZ
# export PointT, PointXYZ, PointXYZRGB, PointXYZRGBA
# export inside, getSubcloud
# export PCLHeader, PointCloud
# export AbstractBoundingBox, AxisAlignedBoundingBox, OrientedBoundingBox
# export getCorners

# bring in the types
include("entities/PCLTypes.jl")
include("entities/OtherTypes.jl")
# bring in further source code
include("services/GeomBasicsUtils.jl")
include("services/PointCloud.jl")
include("services/PointCloudUtils.jl")
include("services/ConsolidateRigidTransform.jl")
include("services/ICP_Simple.jl")


function __init__()
  @require LasIO="570499db-eae3-5eb6-bdd5-a5326f375e68" include("services/LasIOSupport.jl")
  @require RobotOS="22415677-39a4-5241-a37a-00beabbbdae8" include("services/ROSConversions.jl")
  # moved plotting out of Caesar, use Arena.jl or RoMEPlotting.jl instead
end


end