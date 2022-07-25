module Caesar

# import RoME: initfg # collision on RoME.initfg() since no parameters are given in both RoME and Caesar
import Distributions: Normal
import RoME: getRangeKDEMax2D
import IncrementalInference: getSample, initfg
import DistributedFactorGraphs: getManifold

# handy project consts (not for export)
import IncrementalInference: NothingUnion, InstanceType

using Requires
using Dates

using Manifolds
using StaticArrays

import Rotations as _Rot

# TODO remove
const _Rotations = _Rot

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

const CJL = Caesar

# public API exports
include("ExportAPI.jl")


## ===============================================================================================
# and source files
include("BearingRangeTrackingServer.jl")

include("services/_FastTransform3D.jl")
include("SlamServer.jl")
include("DataUtils.jl")
include("UserFunctions.jl")

# Configuration
include("config/CaesarConfig.jl")

include("Deprecated.jl")

# Multisession operation
# include("attic/multisession/Multisession.jl")

# SAS-SLAM
include("beamforming/czt.jl")
include("beamforming/CBF.jl")
include("beamforming/MatchedFilter.jl")
include("beamforming/SASBearing2D.jl")
include("beamforming/SASUtils.jl")

# conditional loading for ROS
function __init__()
  # ZMQ server and endpoints
  @require ZMQ="c2297ded-f4af-51ae-bb23-16f91089e4e1" include("zmq/ZmqCaesar.jl")
  @require PyCall="438e738f-606a-5dbb-bf0a-cddfbfd45ab0" begin
    @info "Loading Caesar PyCall specific utilities (using PyCall)."
    @eval using .PyCall
    @require RobotOS="22415677-39a4-5241-a37a-00beabbbdae8" include("ros/Utils/RosbagSubscriber.jl")
  end
  @require Colors="5ae59095-9a9b-59fe-a467-6f913c188581" include("3rdParty/_PCL/_PCL.jl")
  @require AprilTags="f0fec3d5-a81e-5a6a-8c28-d2b34f3659de" begin 
    include("images/apriltags.jl")
    @require ImageDraw="4381153b-2b60-58ae-a1ba-fd683676385f" include("images/AprilTagDrawingTools.jl")
  end
  @require ImageDraw="4381153b-2b60-58ae-a1ba-fd683676385f" include("images/imagedraw.jl")
  @require ImageMagick="6218d12a-5da1-5696-b52f-db25d2ecc6d1" include("images/imagedata.jl")
  @require Images="916415d5-f1e6-5110-898d-aaa5f9f070e0" begin 
    include("images/images.jl")
    include("images/ScanMatcherUtils.jl")
    include("images/ScanMatcherPose2.jl")
    include("images/ScatterAlignPose2.jl")
    
    @require Gadfly="c91e804a-d5a3-530f-b6f0-dfbca275c004" include("plotting/ScatterAlignPlotting.jl")
  end
  @require Distributed="8ba89e20-285c-5b6f-9357-94700520ee1b" include("images/DistributedUtils.jl")
end

end
