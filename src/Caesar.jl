module Caesar

using Gadfly, RoME, iSAM #, PyLCM

export
  # Victoria Park example -- batch
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
