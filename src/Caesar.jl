module Caesar

# import RoME: initfg # collision on RoME.initfg() since no parameters are given in both RoME and Caesar
import Distributions: Normal
import RoME: getRangeKDEMax2D, getLastPose
import IncrementalInference: getSample, initfg

using Reexport
using Requires
using Dates

@reexport using RoME
@reexport using IncrementalInference
@reexport using KernelDensityEstimate
@reexport using Distributions

using
  Pkg,
  DelimitedFiles,
  Distributed,
  Statistics,
  LinearAlgebra,
  IncrementalInference,
  # Graphs,
  TransformUtils,
  CoordinateTransformations,
  Rotations,
  JSON,
  JSON2,
  FileIO,
  DataStructures,
  ProgressMeter,
  ImageMagick,
  ImageCore,
  DocStringExtensions,
  # CloudGraphs, # TODO: will be movedd to DFG
  # Neo4j, # TODO: will be movedd to DFG
  # Mongoc, # TODO: will be movedd to DFG
  Unmarshal,
  YAML,
  FFTW

export
  GenericInSituSystem,  # insitu components
  makeGenericInSituSys,
  InSituSystem,
  makeInSituSys,
  triggerPose,
  poseTrigAndAdd!,
  processTreeTrackersUpdates!,
  advOdoByRules,
  SLAMWrapper,

  # servers
  tcpStringSLAMServer,
  tcpStringBRTrackingServer,

  # save and load data
  saveSlam, # TODO deprecate
  loadSlam, # TODO deprecate
  haselement,

  # user functions
  identitypose6fg,
  projectrbe,
  hasval,

  # Robot Utils
  getRangeKDEMax2D,

  # sas-slam
  CBFFilterConfig,
  CZTFilter,
  prepCZTFilter,
  getCBFFilter2Dsize,
  constructCBFFilter2D!,
  CBF2D_DelaySum!,
  MatchedFilter,
  SASBearing2D,
  PackedSASBearing2D,
  compare,
  SASDebug,
  reset!,
  prepMF,
  loadConfigFile,
  prepareRangeModel,
  prepareSAS2DFactor,
  wrapRad,
  phaseShiftSingle!,
  liebf!,
  SASDebug



NothingUnion{T} = Union{Nothing, T}

include("BearingRangeTrackingServer.jl")

include("SlamServer.jl")
include("DataUtils.jl")
include("UserFunctions.jl")

# Configuration
include("config/CaesarConfig.jl")


# using CloudGraphs
# include("attic/cloudgraphs/SolverStatus.jl")
# include("attic/cloudgraphs/IterationStatistics.jl")
# include("attic/cloudgraphs/CloudGraphIntegration.jl") # Work in progress code
# include("attic/cloudgraphs/ConvertGeneralSlaminDB.jl")
# include("attic/cloudgraphs/slamindb.jl")
# include("attic/cloudgraphs/MultisessionUtils.jl")
# include("attic/cloudgraphs/FoveationUtils.jl")


# ZMQ server and endpoints
include("zmq/ZmqCaesar.jl")

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
  @require PyCall="438e738f-606a-5dbb-bf0a-cddfbfd45ab0" begin
    @info "Loading Caesar PyCall specific utilities (using PyCall)."
    @eval using .PyCall
    @require RobotOS="22415677-39a4-5241-a37a-00beabbbdae8" begin
      @info "Loading Caesar ROS specific utilities (using RobotOS)."
      include("ros/Utils/RosbagSubscriber.jl")
    end
  end
end

end
