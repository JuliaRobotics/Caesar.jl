module Caesar

using
  IncrementalInference,
  RoME,
  Gadfly,
  Colors,
  JLD,
  HDF5

# using GraphViz, Fontconfig, Cairo, Distributions, DataFrames

export
  # Victoria Park example -- batch
  loadVicPrkDataset,
  addLandmarksFactoGraph!,
  appendFactorGraph!,
  doBatchRun,
  # insitu component
  InSituSystem,
  makeInSituSys,
  triggerPose,
  poseTrigAndAdd!,
  processTreeTrackersUpdates!,
  advOdoByRules,
  progressExamplePlot,
  plotTrckStep




include("VictoriaParkTypes.jl")
# using VictoriaParkTypes
include("VictoriaParkSystem.jl")
include("VicPrkEstimator.jl")


end
