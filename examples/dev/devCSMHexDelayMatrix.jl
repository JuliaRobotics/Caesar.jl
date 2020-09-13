# develop cascade tree init

using Caesar, RoME
using Pkg


## everything in csm2
# testCliqCanRecycled_StateMachine
# testCliqCanIncremtUpdate_StateMachine!
# isCliqUpSolved_StateMachine
# buildCliqSubgraph_StateMachine
# canCliqMargSkipUpSolve_StateMachine
# blockUntilChildrenHaveStatus_StateMachine
# trafficRedirectConsolidate459_StateMachine
# checkIfCliqNullBlock_StateMachine
# doesParentNeedDwn_StateMachine
# determineCliqNeedDownMsg_StateMachine
# slowIfChildrenNotUpSolved_StateMachine

## first steps in csm5
# testCliqCanRecycled_StateMachine
# testCliqCanIncremtUpdate_StateMachine!
# isCliqUpSolved_StateMachine
# buildCliqSubgraph_StateMachine
# canCliqMargSkipUpSolve_StateMachine
# blockUntilChildrenHaveStatus_StateMachine
# trafficRedirectConsolidate459_StateMachine
# checkIfCliqNullBlock_StateMachine
# doesParentNeedDwn_StateMachine
# determineCliqNeedDownMsg_StateMachine
# towardUpOrDwnSolve_StateMachine


allCSMFunctions = [
  exitStateMachine,
  doCliqDownSolve_StateMachine,
  cleanupAfterDownSolve_StateMachine,
  specialCaseRootDownSolve_StateMachine,
  canCliqDownSolve_StateMachine,
  finishCliqSolveCheck_StateMachine,
  mustInitUpCliq_StateMachine,
  doCliqUpSolveInitialized_StateMachine,
  rmUpLikeliSaveSubFg_StateMachine,
  somebodyLovesMe_StateMachine,
  waitChangeOnParentCondition_StateMachine,
  slowOnPrntAsChildrNeedDwn_StateMachine,
  towardUpOrDwnSolve_StateMachine,
  canCliqMargSkipUpSolve_StateMachine,
  attemptDownInit_StateMachine,
  rmMsgLikelihoodsAfterDwn_StateMachine,
  blockUntilSiblingsStatus_StateMachine,
  slowIfChildrenNotUpSolved_StateMachine,
  blockUntilChildrenHaveStatus_StateMachine,
  dwnInitSiblingWaitOrder_StateMachine,
  collectDwnInitMsgFromParent_StateMachine,
  trafficRedirectConsolidate459_StateMachine,
  doAllSiblingsNeedDwn_StateMachine,
  checkIfCliqNullBlock_StateMachine,
  determineCliqNeedDownMsg_StateMachine,
  doAnyChildrenNeedDwn_StateMachine,
  decideUpMsgOrInit_StateMachine,
  doesParentNeedDwn_StateMachine,
  attemptCliqInitUp_StateMachine,
  sendCurrentUpMsg_StateMachine,
  buildCliqSubgraph_StateMachine,
  buildCliqSubgraphForDown_StateMachine,
  isCliqUpSolved_StateMachine,
  checkChildrenAllUpRecycled_StateMachine,
  testCliqCanIncremtUpdate_StateMachine!,
  testCliqCanRecycled_StateMachine,
]


runfile = joinpath(pathof(Caesar) |> dirname |> dirname, "examples", "dev", "devCSMHexDelaysTest.jl")
include(runfile)


for csmDlyFnc in allCSMFunctions, cliqId in 1:6

  injectDelayBefore=[cliqId=>(csmDlyFnc=>5); ]
  # injectDelayBefore=[1=>(exitStateMachine=>0.1); ]
  # injectDelayBefore = nothing

  fg, didFail = csmHexDelayTest(injectDelayBefore)

  fid = open("/tmp/caesar/dbgCSMHexDelayMatrix.log","a")
  println(fid,getLogPath(fg), " -- solve_passed=$(!didFail), ", injectDelayBefore)
  close(fid)

end

#