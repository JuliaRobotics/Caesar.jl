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
csmAnimate
cycleInitByVarOrder!
doautoinit!
drawCliqSubgraphUpMocking
fifoFreeze!
filterHistAllToArray
findRelatedFromPotential
fmcmc!
getClique
getCliqAllVarIds
getCliqAllVarSyms
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
getVariableDim
getVariableInferredDim
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
productpartials!
prodmultiplefullpartials
prodmultipleonefullpartials
resetBuildTreeFromOrder!
resetCliqSolve!
resetData!
resetTreeCliquesForUpSolve!
resetVariable!
sandboxCliqResolveStep
setfreeze!
setValKDE!
setVariableInitialized!
setVariableInferDim!
solveCliq!
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
