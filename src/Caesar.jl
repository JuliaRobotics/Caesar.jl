module Caesar

# import RoME: initfg # collision on RoME.initfg() since no parameters are given in both RoME and Caesar
import Distributions: Normal
import RoME: getRangeKDEMax2D, getLastPose, initfg
import IncrementalInference: batchSolve!

using Reexport

@reexport using RoME
@reexport using KernelDensityEstimate
@reexport using Distributions

using
  Distributed,
  Statistics,
  LinearAlgebra,
  IncrementalInference,
  Graphs,
  TransformUtils,
  CoordinateTransformations,
  Rotations,
  JSON,
  FileIO,
  DataStructures,
  ProgressMeter,
  ImageMagick,
  ImageCore,
  DocStringExtensions,
  CloudGraphs, # TODO: will be movedd to DFG
  Neo4j, # TODO: will be movedd to DFG
  Mongoc # TODO: will be movedd to DFG


export
  GenericInSituSystem,  # insitu components
  makeGenericInSituSys,
  InSituSystem,
  makeInSituSys,
  triggerPose,
  poseTrigAndAdd!,
  processTreeTrackersUpdates!,
  advOdoByRules,
  progressExamplePlot,
  plotPoseDict,
  plotTrckStep,
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

  # repeats from RoME and IIF
  initfg,
  addNode!,
  addFactor!,

  # CloudGraphs helper functions
  insertnodefromcv!,
  checkandinsertedges!,
  getbinarraymongo,
  gettopoint,
  getdotwothree,
  bin2arr,
  fetchsubgraph!,
  getVertNeoIDs!,
  insertrobotdatafirstpose!,
  tryunpackalltypes!,
  fetchrobotdatafirstpose,
  getExVertexNeoIDs,
  db2jld,

  # Robot Utils
  getRangeKDEMax2D,

  # would be CloudGraphs calls
  hasBigDataElement,
  getBigDataElement,
  removeNeo4jID,

  # solver service SLAMinDB
  getcredentials,
  startSlamInDb,
  runSlamInDbOnSession,
  slamindb,
  convertdb,
  resetconvertdb,
  getmaxfactorid,

  # webserver
  SolverStatus,
  CaesarConfig,
  IterationStatistics,
  VisualizationConfig,

  # multisession utils
  multisessionquery,
  parsemultisessionqueryresult!,
  getLandmOtherSessNeoIDs,
  getAllLandmarkNeoIDs,
  getLocalSubGraphMultisession,
  findExistingMSConstraints,
  getprpt2kde,
  rmInstMultisessionPriors!,
  removeMultisessions!

NothingUnion{T} = Union{Nothing, T}

include("BearingRangeTrackingServer.jl")

include("SlamServer.jl")
include("DataUtils.jl")
include("UserFunctions.jl")

# Configuration
include("config/CaesarConfig.jl")


# using CloudGraphs
include("cloudgraphs/SolverStatus.jl")
include("cloudgraphs/IterationStatistics.jl")
include("cloudgraphs/CloudGraphIntegration.jl") # Work in progress code
include("cloudgraphs/ConvertGeneralSlaminDB.jl")
include("cloudgraphs/slamindb.jl")
include("cloudgraphs/MultisessionUtils.jl")
include("cloudgraphs/FoveationUtils.jl")

end
