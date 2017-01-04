module Caesar

using
  RoME,
  IncrementalInference,
  Graphs,
  KernelDensityEstimate,
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
  JSON
  #CloudGraphs

# using GraphViz, Fontconfig, Cairo, Distributions, DataFrames

export
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

  # cloudgraph
  usecloudgraphsdatalayer!,
  # CloudGraph stuff
  registerGeneralVariableTypes!,
  fullLocalGraphCopy!,
  removeGenericMarginals!,
  setBackendWorkingSet!,
  setDBAllReady!,
  getExVertFromCloud,
  getAllExVertexNeoIDs,
  getPoseExVertexNeoIDs,
  copyAllNodes!,
  copyAllEdges!,
  registerCallback!,

  # drawing functions
  VisualizationContainer,
  startdefaultvisualization,
  newtriad!,
  visualizetriads,
  visualizeallposes!,
  visualizeDensityMesh!,

  # for models
  loadmodel,
  DrawModel,
  DrawROV,

  # user functions
  identitypose6fg,
  projectrbe,
  solveandvisualize



# include("CloudGraphIntegration.jl") # Work in progress code

include("VictoriaParkTypes.jl")
# using VictoriaParkTypes
include("VictoriaParkSystem.jl")
# include("VicPrkEstimator.jl")

include("SlamServer.jl")
include("BearingRangeTrackingServer.jl")

include("DataUtils.jl")

include("VisualizationUtils.jl")
include("ModelVisualizationUtils.jl")

include("UserFunctions.jl")

end
