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
  ImageMagick, # figure out who else is using this and move to requires
  ImageCore,
  DocStringExtensions,
  Unmarshal,
  YAML,
  FFTW,
  TimeZones,
  TensorCast

using Optim

using Reexport


# import RoME: initfg # collision on RoME.initfg() since no parameters are given in both RoME and Caesar
import RoME: getRangeKDEMax2D
import IncrementalInference: getSample, initfg
import DistributedFactorGraphs: getManifold

# handy project consts (not for export)
import IncrementalInference: NothingUnion, InstanceType
import GeometricalPredicates as GeoPr



# const CJL = Caesar

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

# SAS-SLAM
include("beamforming/czt.jl")
include("beamforming/CBF.jl")
include("beamforming/MatchedFilter.jl")
include("beamforming/SASBearing2D.jl")
include("beamforming/SASUtils.jl")

include("3rdParty/_PCL/_PCL.jl")

# object affordance work
include("objects/ObjectAffordanceSubcloud.jl")

# ImageDraw functionality, used by many extensions and therefore a regular (but minimum able) dependency
include("images/imagedraw.jl")

# weakdeps
include("../ext/factors/Pose2AprilTag4Corners.jl")
include("../ext/WeakdepsPrototypes.jl")

# standardized code deprecation
include("Deprecated.jl")


# FIXME remove
using Requires


# conditional loading for ROS
function __init__()
  # ZMQ server and endpoints
  # @require ZMQ="c2297ded-f4af-51ae-bb23-16f91089e4e1" include("zmq/ZmqCaesar.jl")
  @require PyCall="438e738f-606a-5dbb-bf0a-cddfbfd45ab0" begin
    @require RobotOS="22415677-39a4-5241-a37a-00beabbbdae8" include("ros/CaesarROS.jl")
  end
  # @require Colors="5ae59095-9a9b-59fe-a467-6f913c188581" include("3rdParty/_PCL/_PCL.jl")
  # @require AprilTags="f0fec3d5-a81e-5a6a-8c28-d2b34f3659de" begin 
  #   include("images/apriltags.jl")
  #   @require ImageDraw="4381153b-2b60-58ae-a1ba-fd683676385f" include("images/AprilTagDrawingTools.jl")
  # end
  # @require ImageDraw="4381153b-2b60-58ae-a1ba-fd683676385f" include("images/imagedraw.jl")
  @require ImageMagick="6218d12a-5da1-5696-b52f-db25d2ecc6d1" include("images/imagedata.jl")
  @require Images="916415d5-f1e6-5110-898d-aaa5f9f070e0" begin 
    include("images/images.jl")
    include("images/ImageToVideoUtils.jl")
    include("images/ScanMatcherUtils.jl")
    include("images/ScanMatcherPose2.jl")
    include("images/ScatterAlignPose2.jl")
    
    # moved Gadfly plotting to RoMEPlotting
    # @require Gadfly="c91e804a-d5a3-530f-b6f0-dfbca275c004" include("plotting/ScatterAlignPlotting.jl")
    @require RobotOS="22415677-39a4-5241-a37a-00beabbbdae8" include("images/ROSConversions.jl")
  end
  @require ImageFeatures="92ff4b2b-8094-53d3-b29d-97f740f06cef" include("images/imagefeatures.jl")
  @require Distributed="8ba89e20-285c-5b6f-9357-94700520ee1b" include("images/DistributedUtils.jl")
end

end
