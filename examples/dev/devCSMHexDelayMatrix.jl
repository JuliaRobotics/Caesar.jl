# develop cascade tree init

using Caesar, RoME
using Pkg


## everything in csm2
# canCliqMargRecycle_StateMachine
# canCliqIncrRecycle_StateMachine
# isCliqUpSolved_StateMachine
# buildCliqSubgraph_StateMachine
# canCliqMargSkipUpSolve_StateMachine
# blockUntilChildrenHaveStatus_StateMachine
# trafficRedirectConsolidate459_StateMachine
# maybeNeedDwnMsg_StateMachine
# doesParentNeedDwn_StateMachine
# determineCliqNeedDownMsg_StateMachine
# slowIfChildrenNotUpSolved_StateMachine

## first steps in csm5
# canCliqMargRecycle_StateMachine
# canCliqIncrRecycle_StateMachine
# isCliqUpSolved_StateMachine
# buildCliqSubgraph_StateMachine
# canCliqMargSkipUpSolve_StateMachine
# blockUntilChildrenHaveStatus_StateMachine
# trafficRedirectConsolidate459_StateMachine
# maybeNeedDwnMsg_StateMachine
# doesParentNeedDwn_StateMachine
# determineCliqNeedDownMsg_StateMachine
# towardUpOrDwnSolve_StateMachine


allCSMFunctions = [
  doCliqDownSolve_StateMachine,
  cleanupAfterDownSolve_StateMachine,
  specialCaseRootDownSolve_StateMachine,
  canCliqDownSolve_StateMachine,
  checkUpsolveFinished_StateMachine,
  prepInitUp_StateMachine,
  doCliqUpSolveInitialized_StateMachine,
  rmUpLikeliSaveSubFg_StateMachine,
  wipRedirect459Dwn_StateMachine,
  waitChangeOnParentCondition_StateMachine,
  slowOnPrntAsChildrNeedDwn_StateMachine,
  towardUpOrDwnSolve_StateMachine,
  canCliqMargSkipUpSolve_StateMachine,
  tryDwnInitCliq_StateMachine,
  rmMsgLikelihoodsAfterDwn_StateMachine,
  blockSiblingStatus_StateMachine,
  slowIfChildrenNotUpSolved_StateMachine,
  blockUntilChildrenHaveStatus_StateMachine,
  dwnInitSiblingWaitOrder_StateMachine,
  collectDwnInitMsgFromParent_StateMachine,
  trafficRedirectConsolidate459_StateMachine,
  doAllSiblingsNeedDwn_StateMachine,
  maybeNeedDwnMsg_StateMachine,
  determineCliqNeedDownMsg_StateMachine,
  decideUpMsgOrInit_StateMachine,
  doesParentNeedDwn_StateMachine,
  attemptCliqInitUp_StateMachine,
  sendCurrentUpMsg_StateMachine,
  buildCliqSubgraph_StateMachine,
  buildCliqSubgraphForDown_StateMachine,
  isCliqUpSolved_StateMachine,
  checkChildrenAllUpRecycled_StateMachine,
  canCliqIncrRecycle_StateMachine,
  canCliqMargRecycle_StateMachine,
  exitStateMachine,
]


runfile = joinpath(pathof(Caesar) |> dirname |> dirname, "examples", "dev", "devCSMHexDelaysTest.jl")
include(runfile)


for csmDlyFnc in allCSMFunctions, cliqId in 1:6

  injectDelayBefore=[cliqId=>(csmDlyFnc=>5); ]
  # injectDelayBefore=[1=>(exitStateMachine=>0.1); ]
  # injectDelayBefore = nothing

  fg, didFail = csmHexDelayTest(injectDelayBefore)

  fid = open("/tmp/caesar/dbgCSMHexDelayMatrix2.log","a")
  println(fid,getLogPath(fg), " -- solve_passed=$(!didFail), ", injectDelayBefore)
  close(fid)

end

#