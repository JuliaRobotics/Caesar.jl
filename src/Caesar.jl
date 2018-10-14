module Caesar

# import RoME: initfg # collision on RoME.initfg() since no parameters are given in both RoME and Caesar
import Distributions: Normal
import RoME: getRangeKDEMax2D, getLastPose, initfg
import IncrementalInference: batchSolve!

using Reexport

@reexport using RoME

using
  IncrementalInference,
  Graphs,
  KernelDensityEstimate,
  Distributions,
  TransformUtils,
  CoordinateTransformations,
  Rotations,
  JLD,
  HDF5,
  JSON,
  FileIO,
  DataStructures,
  ProgressMeter,
  ImageMagick,
  ImageCore,
  DocStringExtensions,
  CloudGraphs,
  Neo4j,
  Mongo,
  LibBSON,
  Unmarshal


export
  # pass through from KDE
  kde!,
  getPoints,
  getBW,
  Ndim,
  Npts,

  # pass through from IIF and RoME
  ls,
  FactorGraph,
  writeGraphPdf,
  getVert,
  getVal,
  saveplot,
  wipeBuildNewTree!,
  inferOverTree!,
  inferOverTreeR!,
  # callbacks for datalayer changes
  localapi,
  dlapi,
  # Victoria Park example -- batch
  loadVicPrkDataset,

  # passthrough variable and factor types
  Pose2,
  Point2,

  # passthrough RoME factor types
  PriorPose2,
  Pose2Pose2,
  Pose2DPoint2DBearingRange, # deprecated
  Pose2Point2BearingRange, # deprecated

  # insitu component
  GenericInSituSystem,
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
  saveSlam,
  loadSlam,
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

VoidUnion{T} = Union{Void, T}

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

# these are the ZMQ endpoints
# include("endpoints/Models.jl")
include("endpoints/Services.jl")
include("endpoints/models/config.jl")
include("endpoints/models/session.jl")
include("endpoints/services/config.jl")
include("endpoints/services/session.jl")

end
