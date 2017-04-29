module Caesar

instpkg = Pkg.installed();

# import RoME: initfg # collision on RoME.initfg() since no parameters are given in both RoME and Caesar
import Distributions: Normal
import DrakeVisualizer: Triad
import RoME: getRangeKDEMax2D, getLastPose

using
  RoME,
  IncrementalInference,
  Graphs,
  KernelDensityEstimate,
  Distributions,
  DrakeVisualizer,
  TransformUtils,
  CoordinateTransformations,
  GeometryTypes,
  Rotations,
  Gadfly,
  Colors,
  ColorTypes,
  JLD,
  HDF5,
  JSON,
  MeshIO,
  FileIO,
  NLsolve,
  DataStructures,
  ProgressMeter,
  ImageMagick,
  ImageCore,
  PyCall

if haskey(instpkg,"CloudGraphs")
  using CloudGraphs
  using Neo4j
  using Mongo
  using LibBSON
end

# using GraphViz, Fontconfig, Cairo, Distributions, DataFrames




export
  # pass through from KDE
  kde!,
  plotKDE,
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
  # callbacks for datalayer changes
  localapi,
  dlapi,
  # Victoria Park example -- batch
  loadVicPrkDataset,

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

  # drawing functions
  VisualizationContainer,
  startdefaultvisualization,
  newtriad!,
  visualizetriads,
  visualizeallposes!,
  visualizeDensityMesh!,
  updaterealtime!,
  visualizerealtime,
  # new tree interface
  drawpose!,
  drawposepoints!,
  drawLine!,
  drawLineBetween!,
  drawAllOdometryEdges!,
  pointToColor,
  findAllBinaryFactors,
  drawAllBinaryFactorEdges!,

  # for models
  loadmodel,
  DrawModel,
  DrawROV,
  DrawScene,
  #deleting functions
  deletemeshes!,

  # more drawing utils
  ArcPointsRangeSolve,
  findaxiscenter!,
  parameterizeArcAffineMap,
  animatearc,

  # vis service
  # drawdbsession,
  drawdbdirector,
  meshgrid,
  DepthCamera,
  buildmesh!,
  reconstruct,

  # user functions
  identitypose6fg,
  projectrbe,
  solveandvisualize,
  hasval,

  # repeats from RoME and IIF
  initfg,
  addNode!,
  addFactor!,

  # Using CloudGraphs
  # install cloudgraphs
  installcloudgraphs,
  # helper functions
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

  # Robot Utils
  getRangeKDEMax2D,

  # would be CloudGraphs calls
  hasBigDataElement,
  getBigDataElement,
  removeNeo4jID,

  # solver service SLAMinDB
  getcredentials,
  slamindb,
  convertdb,
  resetconvertdb,
  getmaxfactorid,

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

typealias VoidUnion{T} Union{Void, T}

include("BearingRangeTrackingServer.jl")

include("SlamServer.jl")
include("DataUtils.jl")
include("VisualizationUtils.jl")
include("ModelVisualizationUtils.jl")


include("BearingRangeTrackingServer.jl")

include("SlamServer.jl")
include("DataUtils.jl")
include("VisualizationUtils.jl")
include("ModelVisualizationUtils.jl")
include("UserFunctions.jl")

if haskey(instpkg, "CloudGraphs")
  include("cloudgraphs/CloudGraphIntegration.jl") # Work in progress code
  include("cloudgraphs/ConvertGeneralSlaminDB.jl")
  include("cloudgraphs/slamindb.jl")
  include("cloudgraphs/DBVisualizationUtils.jl")
  include("cloudgraphs/DirectorVisService.jl")
  include("cloudgraphs/MultisessionUtils.jl")
  include("cloudgraphs/ImageUtils.jl")
  include("cloudgraphs/FoveationUtils.jl")
end


end
