module Caesar

using Gadfly, RoME, iSAM #, PyLCM

using JLD, HDF5
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

include("VictoriaParkSystem.jl")
include("VicPrkEstimator.jl")



end
