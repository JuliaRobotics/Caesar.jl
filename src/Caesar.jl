module Caesar


import Distributions: Normal # TODO, upstream or remove?

using Dates
using Colors
using StaticArrays

using Manifolds
import Rotations as _Rot

const _Rotations = _Rot # TODO remove

import GeometryBasics as GeoB

using ImageDraw

using
  Pkg,
  DelimitedFiles,
  Distributed,
  Statistics,
  LinearAlgebra,
  IncrementalInference,
  TransformUtils,
  CoordinateTransformations,
  JSON,
  JSON2,
  JSON3,
  UUIDs,
  Base64,
  FileIO,
  DataStructures,
  ProgressMeter,
  ImageCore,
  DocStringExtensions,
  Unmarshal,
  YAML,
  FFTW,
  TimeZones,
  TensorCast

using Optim

using Reexport
using PrecompileTools


# import RoME: initfg # collision on RoME.initfg() since no parameters are given in both RoME and Caesar
import RoME: getRangeKDEMax2D
import IncrementalInference: getSample, initfg
import DistributedFactorGraphs: getManifold

# handy project consts (not for export)
import IncrementalInference: NothingUnion, InstanceType
import GeometricalPredicates as GeoPr


# public API exports
include("ExportAPI.jl")

## ===============================================================================================
# and source files
include("services/BearingRangeTrackingServer.jl")

include("transforms/entities/TransformTypes.jl")
include("transforms/services/HomographyTransforms.jl")
include("transforms/services/_FastTransform3D.jl")

include("entities/OtherTypes.jl")
include("services/WorkflowBlobNames.jl")
include("services/PointUtils.jl")
include("services/DataUtils.jl")
include("services/UserFunctions.jl")
include("services/ToImage.jl")

# SAS-SLAM
include("beamforming/czt.jl")
include("beamforming/CBF.jl")
include("beamforming/MatchedFilter.jl")
include("beamforming/SASBearing2D.jl")
include("beamforming/SASUtils.jl")

# Bag of words
include("bagofwords/BagOfWords.jl")

# manual type-implementation of Point Cloud Library
include("3rdParty/_PCL/_PCL.jl")

# object affordance work
include("objects/ObjectAffordanceSubcloud.jl")

# ImageDraw functionality, used by many extensions and therefore a regular (but minimum able) dependency
include("images/imagedraw.jl")

# experimentals
include("dev/FolderDict.jl")

# weakdeps
include("../ext/factors/Pose2AprilTag4Corners.jl")
include("../ext/factors/ScanMatcherPose2.jl")
include("../ext/factors/ScatterAlignPose.jl")
include("../ext/WeakdepsPrototypes.jl")

# standardized code deprecation
include("Deprecated.jl")


@compile_workload begin
  # In here put "toy workloads" that exercise the code you want to precompile
  # warmUpSolverJIT()
end

end
