
@info "Caesar.jl is loading (but not exporting) tools requiring Colors.jl; e.g. Caesar._PCL"

module _PCL

using ..Colors
import ..Caesar: _FastTransform3D

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

# Going to add dispatches on these functions
import Base: getindex, setindex!, resize!, cat, convert, sizeof, hasproperty, getproperty

# gets overloaded
import Manifolds: apply
import IncrementalInference: ArrayPartition

## hold off on exports, users can in the mean-time use/import via e.g. _PCL.PointXYZ
# export PointT, PointXYZ, PointXYZRGB, PointXYZRGBA
# export PCLHeader, PointCloud


# bring in the types
include("entities/PCLTypes.jl")
# bring in further source code
include("services/PointCloud.jl")
include("services/ICP_Simple.jl")


function __init__()
  @require RobotOS="22415677-39a4-5241-a37a-00beabbbdae8" include("services/ROSConversions.jl")
end


end