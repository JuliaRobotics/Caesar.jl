# new Sandshark example
# add more julia processes
# using Distributed
# nprocs() < 4 ? addprocs(4-nprocs()) : nothing

using Caesar, RoME
# @everywhere using Caesar, RoME
using Interpolations
using Distributions

using RoMEPlotting
using Gadfly, DataFrames
using ProgressMeter
using DelimitedFiles
using Logging

# const TU = TransformUtils

Gadfly.set_default_plot_size(35cm,25cm)

include(joinpath(@__DIR__,"Plotting.jl"))
include(joinpath(@__DIR__,"SandsharkUtils.jl"))


# Step: Selecting a subset for processing and build up a cache of the factors.
epochs = timestamps[50:2:60]
lastepoch = 0
for ep in epochs
  global lastepoch
  if lastepoch != 0
    # @show interp_yaw(ep)
    deltaAng = interp_yaw(ep) - interp_yaw(lastepoch)

    wXi = TU.SE2([interp_x(lastepoch);interp_y(lastepoch);interp_yaw(lastepoch)])
    wXj = TU.SE2([interp_x(ep);interp_y(ep);interp_yaw(ep)])
    iDXj = se2vee(wXi\wXj)
    NAV[ep] = iDXj
    # println("$(iDXj[1]), $(iDXj[2]), $(iDXj[3])")

    odoDict[ep] = Pose2Pose2(MvNormal(NAV[ep], Matrix(Diagonal([0.1;0.1;0.005].^2))))
  end
  rangepts = rangedata[ep][:]
  rangeprob = kde!(rangepts)
  azipts = azidata[ep][:,1]
  aziprob = kde!(azipts)

  # prep the factor functions
  ppbrDict[ep] = Pose2Point2BearingRange(aziprob, rangeprob)
  lastepoch = ep
end


## Step: Building the factor graph
fg = initfg()
# Add a central beacon with a prior
addVariable!(fg, :l1, Point2)
# Pinger location is (0.6; -16)
addFactor!(fg, [:l1], PriorPose2( MvNormal([0.6; -16], Matrix(Diagonal([0.1; 0.1].^2)) ) ), autoinit=false)

index = 0
for ep in epochs
    global index
    curvar = Symbol("x$index")
    addVariable!(fg, curvar, Pose2)
    if ep != epochs[1]
      # Odo factor x(i-1) -> xi
      addFactor!(fg, [Symbol("x$(index-1)"); curvar], odoDict[ep], autoinit=false)
    else
      # Prior to the first pose location (a "GPS" prior)
      initLoc = [interp_x(ep);interp_y(ep);interp_yaw(ep)]
      println("Adding a prior at $curvar, $initLoc")
      addFactor!(fg, [curvar], PriorPose2( MvNormal(initLoc, Matrix(Diagonal([0.1;0.1;0.05].^2))) ), autoinit=false)
    end
    index+=1
end


# Just adding the first one...
# addFactor!(fg, [:x0; :l1], ppbrDict[epochs[1]], autoinit=false)
addFactor!(fg, [:x5; :l1], ppbrDict[epochs[6]], autoinit=false)


# Debugging options
getSolverParams(fg).drawtree = true
getSolverParams(fg).showtree = true
getSolverParams(fg).async = true
getSolverParams(fg).downsolve = false
getSolverParams(fg).multiproc = false
getSolverParams(fg).limititers = 30


tree, smt, hist = solveTree!(fg, recordcliqs=ls(fg))


drawPosesLandms(fg, meanmax=:max)

0


# assignTreeHistory!(tree, hist)
#
# csmAnimate(fg, tree, ls(fg), frames=1000) #[:x0; :x2]
# # Base.rm("/tmp/caesar/csmCompound/out.ogv")
# run(`ffmpeg -r 10 -i /tmp/caesar/csmCompound/csm_%d.png -c:v libtheora -vf fps=25 -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" -q 10 /tmp/caesar/csmCompound/out.ogv`)
# @async run(`totem /tmp/caesar/csmCompound/out.ogv`)








# getVariableInferredDim(fg, :x0)
# getVariableInferredDim(fg, :x1)
# getVariableInferredDim(fg, :x2)
# getVariableInferredDim(fg, :x3)
# getVariableInferredDim(fg, :x4)
# getVariableInferredDim(fg, :x5)
# getVariableInferredDim(fg, :l1)




## OR Do it manually??
tree = wipeBuildNewTree!(fg)
drawTree(tree, show=true)

smt, hist = solveCliq!(fg, tree, :x0)
smt, hist = solveCliq!(fg, tree, :l1)

smt, hist = solveCliq!(fg, tree, :x4)
smt, hist = solveCliq!(fg, tree, :x2)
smt, hist = solveCliq!(fg, tree, :x1)




### DEV work below


cliq = getClique(tree,:x0)

# getCliqVariableInferredPercent(fg, cliq)








### Pretend solve cliq 2


cliq = getClique(tree, :x2)
syms = getCliqAllVarSyms(fg, cliq)
c3sfg = buildSubgraphFromLabels(fg, syms)
# c2sfg = buildSubgraphFromLabels(fg, syms)



dfg = fg
@info "8a, needs down message -- attempt down init"
prnt = getParent(tree, cliq)[1]

# take atomic lock when waiting for down ward information
# lockUpStatus!(getData(prnt))

dwinmsgs = prepCliqInitMsgsDown!(dfg, tree, prnt, logger=ConsoleLogger()) # cliqSubFg
dwnkeys = collect(keys(dwinmsgs))

## DEVIdea
msgfcts = addMsgFactors!(c3sfg, dwinmsgs)
# writeGraphPdf(c3sfg, show=true)


@info "8a, attemptCliqInitD., dwinmsgs=$(dwnkeys)"

# DEVidea
# sdims = getCliqVariableMoreInitDims(c3sfg, cliq) # removed after IIF v0.25
updateCliqSolvableDims!(cliq, sdims)

# determine if more info is needed for partial
# priorize solve order for mustinitdown with lowest dependency first
# follow example from issue #344
mustwait = false
if length(intersect(dwnkeys, getCliqSeparatorVarIds(cliq))) == 0 # length(dwinmsgs) == 0 ||
  @info "8a, attemptCliqInitDown_StateMachine, no can do, must wait for siblings to update parent first."
  global mustwait = true
elseif getSiblingsDelayOrder(tree, cliq, prnt, dwinmsgs, logger=ConsoleLogger())
  @info "8a, attemptCliqInitD., prioritize"
  global mustwait = true
elseif getCliqSiblingsPartialNeeds(tree, cliq, prnt, dwinmsgs, logger=ConsoleLogger())
  @info "8a, attemptCliqInitD., partialneedsmore"
  global mustwait = true
end


# remove the downward messages too
deleteMsgFactors!(c3sfg, msgfcts)

# unlock
# @info "8a, attemptCliqInitD., unlockUpStatus!"
# unlockUpStatus!(getData(prnt))









fetchCliqSolvableDims(getClique(tree, :x4))
fetchCliqSolvableDims(getClique(tree, :x2))
fetchCliqSolvableDims(getClique(tree, :x0))






prnt = getClique(tree, :x1)
getCliqSiblingsPriorityInitOrder(tree, prnt)

# getData(getClique(tree, :x0)).solvableDims

# fetch(getData(getClique(tree, :x0)).solvableDims)

#
