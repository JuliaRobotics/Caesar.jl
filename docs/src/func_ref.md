# Additional Function Reference

```@contents
Pages = [
    "func_ref.md"
]
Depth = 3
```

## RoME

```@docs
getRangeKDEMax2D
initFactorGraph!
addOdoFG!
```

## IncrementalInference
```@docs
approxCliqMarginalUp!
areCliqVariablesAllMarginalized
attemptTreeSimilarClique
childCliqs
cliqHistFilterTransitions
cycleInitByVarOrder!
doautoinit!
drawCliqSubgraphUpMocking
fifoFreeze!
filterHistAllToArray
fmcmc!
getClique
getCliqAllVarIds
getCliqAssocMat
getCliqDepth
getCliqDownMsgsAfterDownSolve
getCliqFrontalVarIds
getCliqVarInitOrderUp
getCliqMat
getCliqSeparatorVarIds
getCliqSiblings
getCliqVarIdsPriors
getCliqVarSingletons
getParent
getTreeAllFrontalSyms
hasClique
isInitialized
isMarginalized
isTreeSolved
isPartial
localProduct
makeCsmMovie
parentCliq
predictVariableByFactor
printCliqHistorySummary
resetCliqSolve!
resetData!
resetTreeCliquesForUpSolve!
resetVariable!
setfreeze!
setValKDE!
setVariableInitialized!
solveCliqWithStateMachine!
transferUpdateSubGraph!
treeProductDwn
treeProductUp
unfreezeVariablesAll!
dontMarginalizeVariablesAll!
updateFGBT!
upGibbsCliqueDensity
resetVariableAllInitializations!
```
