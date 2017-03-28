module Caesar

instpkg = Pkg.installed();

# import RoME: initfg
import Distributions: Normal
import DrakeVisualizer: Triad

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
  # pass through from IIF and RoME
  ls,
  # Victoria Park example -- batch
  loadVicPrkDataset,
  # addLandmarksFactoGraph!,
  # appendFactorGraph!,
  # doBatchRun,
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

  # more passthrough
  initfg,

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
  drawdbsession,
  drawdbdirector,
  meshgrid,
  DepthCamera,
  buildmesh!,
  reconstruct,

  # user functions
  identitypose6fg,
  projectrbe,
  solveandvisualize,

  # repeats from RoME and IIF
  initfg,
  addNode!,
  addFactor!,

  # Using CloudGraphs
  # install cloudgraphs
  installcloudgraphs,
  # helper functions
  getbinarraymongo,
  gettopoint,
  getdotwothree,
  bin2arr,

  # would be CloudGraphs calls
  hasBigDataElement,
  getBigDataElement,

  # solver service SLAMinDB
  getcredentials,
  slamindb,
  convertdb,
  resetconvertdb



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
end


end
