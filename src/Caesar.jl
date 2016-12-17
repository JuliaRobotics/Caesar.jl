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
  Gadfly,
  Colors,
  JLD,
  HDF5,
  JSON,
  CloudGraphs

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
  registerCallback!



include("CloudGraphIntegration.jl") # Work in progress code

include("VictoriaParkTypes.jl")
# using VictoriaParkTypes
include("VictoriaParkSystem.jl")
# include("VicPrkEstimator.jl")

include("SlamServer.jl")
include("BearingRangeTrackingServer.jl")

include("DataUtils.jl")

end
