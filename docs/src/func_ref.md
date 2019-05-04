## Function Reference

```@contents
Pages = [
    "func_ref.md"
]
Depth = 3
```

**WORK IN PROGRESS**  Not all functions have been added to this directory yet.

### Caesar
```@docs
appendvertbigdata!
consoleaskuserfordb
db2jld
executeQuery
fetchrobotdatafirstpose
fetchsubgraph!
findExistingMSConstraints
getAllLandmarkNeoIDs
getBigDataElement
getExVertexNeoIDs
getLandmOtherSessNeoIDs
getLocalSubGraphMultisession
getPoseExVertexNeoIDs
getVertNeoIDs!
getfirstpose
getnewvertdict
getprpt2kde
hasBigDataElement
insertrobotdatafirstpose!
removeNeo4jID
resetentireremotesession
rmInstMultisessionPriors!
standardcloudgraphsetup
updatenewverts!
whosNear2D
whosNear3D
```

### RoME

```@docs
getRangeKDEMax2D
initFactorGraph!
addOdoFG!
```

### IncrementalInference
```@docs
addVariable!
addFactor!
allnums
approxCliqMarginalUp!
approxConv
areCliqChildrenNeedDownMsg
areCliqVariablesAllMarginalized
batchSolve!
blockCliqUntilChildrenHaveUpStatus
buildSubgraphFromLabels
buildTreeFromOrdering!
childCliqs
cliqGibbs
cliqInitSolveUp!
compareAllVariables
compareFactorGraphs
compareSimilarFactors
compareSimilarVariables
compareSubsetFactorGraph
compareVariable
convert2packedfunctionnode
cycleInitByVarOrder!
decodefg
deleteFactor!
deleteVariable!
doautoinit!
doCliqAutoInitUp!
doCliqUpSolve!
downGibbsCliqueDensity
downMsgPassingRecursive
dwnMsg
encodefg
fifoFreeze!
findRelatedFromPotential
fmcmc!
getCurrentWorkspaceFactors
getCurrentWorkspaceVariables
getCliq
getCliqOrderUpSolve
getCliqAllVarIds
getCliqAllVarSyms
getCliqAssocMat
getCliqChildMsgsUp
getCliqFrontalVarIds
getCliqInitVarOrderUp
getCliqMat
getCliqOrderUpSolve
getCliqSeparatorVarIds
getCliqStatusUp
getCliqVarIdsPriors
getCliqVars
getCliqVarSingletons
getData
getFactor
getKDE
getManifolds
getMaxVertId
getParent
getSofttype
getTreeCliqSolveOrderUp
getVal
getVariable
getVertKDE
getUpMsgs
getDwnMsgs
initfg
initInferTreeUp!
isCliqMarginalizedFromVars
isCliqReadyInferenceUp
isFactor
isInitialized
isMarginalized
isTreeSolved
isPartial
isVariable
landmarks
loadjld
localProduct
ls
lsf
lsRear
manualinit!
parentCliq
packFromLocalPotentials!
prepBatchTree!
prepCliqInitMsgsDown!
prepCliqInitMsgsUp!
printgraphmax
productpartials!
prodmultiplefullpartials
prodmultipleonefullpartials
resetData!
resetTreeCliquesForUpSolve!
savejld
setCliqAsMarginalized!
setCliqStatus!
setDwnMsg!
setfreeze!
setTreeCliquesMarginalized!
setUpMsg!
subgraphFromVerts
treeProductDwn
treeProductUp
updateFGBT!
updateTreeCliquesAsMarginalizedFromVars!
upGibbsCliqueDensity
upMsg
writeGraphPdf
showFactor
```

<!-- IIF v0.5.3 -->
<!-- showVariable -->
