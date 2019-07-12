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


# first solve and initialization
getSolverParams(fg).drawtree = true
getSolverParams(fg).showtree = true
getSolverParams(fg).async = true
# getSolverParams(fg).downsolve = false


tree, smt, hist = solveTree!(fg, recordcliqs=ls(fg))


# # now solve the second portion of the tree.
# getSolverParams(fg).downsolve = true
# tree, smt, hist = solveTree!(fg, tree, recordcliqs=ls(fg))


# assignTreeHistory!(tree, hist)
# csmAnimate(fg, tree, [:x0; :l1])
# Base.rm("/tmp/caesar/csmCompound/out.ogv")
# run(`ffmpeg -r 10 -i /tmp/caesar/csmCompound/csm_%d.png -c:v libtheora -vf fps=25 -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" -q 10 /tmp/caesar/csmCompound/out.ogv`)
# run(`totem /tmp/caesar/csmCompound/out.ogv`)


# writeGraphPdf(fg, show=true)

drawPosesLandms(fg)


fsy = getTreeAllFrontalSyms(fg, tree)

@show fsy
printCliqHistorySummary(tree, fsy[1])
printCliqHistorySummary(tree, fsy[2])
printCliqHistorySummary(tree, fsy[3])
printCliqHistorySummary(tree, fsy[4])
printCliqHistorySummary(tree, fsy[5])
0



# #
# using Profile, ProfileView
#
# # do this twice
# Profile.clear()
# @profile stuff = sandboxCliqResolveStep(tree,:l1,8)
# #
# ProfileView.view()
# #
# Juno.profiler()


csmAnimate(fg, tree, fsy, frames=1000)
# Base.rm("/tmp/caesar/csmCompound/out.mp4")
run(`ffmpeg -r 10 -i /tmp/caesar/csmCompound/csm_%d.png -c:v libx264 -vf fps=25 -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" /tmp/caesar/csmCompound/out.mp4`)
run(`vlc /tmp/caesar/csmCompound/out.mp4`)


# animateCliqStateMachines(tree,fsy,frames=100)



## the success case x4 second last and x1 last

# smt, hist = solveCliq!(fg, tree, :x0)

using Dates

tree = wipeBuildNewTree!(fg, drawpdf=true, show=true)
fsy = getTreeAllFrontalSyms(fg, tree)

resetTreeCliquesForUpSolve!(tree)
setTreeCliquesMarginalized!(fg, tree)

# queue all the tasks
alltasks = Vector{Task}(undef, length(tree.cliques))
cliqHistories = Dict{Int,Vector{Tuple{DateTime, Int, Function, CliqStateMachineContainer}}}()


# preempt X4 10 steps so that X2 can fully solve first
idx = whichCliq(tree, :x4).index
stf_x4 = IIF.tryCliqStateMachineSolve!(fg, tree, idx, cliqHistories, limititers=10, drawtree=true, N=100, recordcliqs=fsy)
# t_x4 = @async

idx = whichCliq(tree, :x0).index
stf_x0 = IIF.tryCliqStateMachineSolve!(fg, tree, idx, cliqHistories, drawtree=true, N=100, recordcliqs=fsy)
# t_x0 = @async

idx = whichCliq(tree, :l1).index
stf_l1 = IIF.tryCliqStateMachineSolve!(fg, tree, idx, cliqHistories, drawtree=true, N=100, recordcliqs=fsy)
# t_l1 = @async




idx = whichCliq(tree, :x2).index
stf_x2 = IIF.tryCliqStateMachineSolve!(fg, tree, idx, cliqHistories, drawtree=true, N=100, recordcliqs=fsy) # , limititers=14
# t_x2 = @async


idx = whichCliq(tree, :x4).index
@async stf_x4 = IIF.tryCliqStateMachineSolve!(fg, tree, idx, cliqHistories, drawtree=true, N=100, recordcliqs=fsy) # , limititers=8
# t_x4 = @async


#
# drawTree(tree)
# # partial block on :x4 should return true
# cliq = whichCliq(tree, :x4)
# prnt = getParent(tree, cliq)[1]
# dwinmsgs = prepCliqInitMsgsDown!(fg, tree, prnt)
# getCliqSiblingsPartialNeeds(tree, cliq, prnt, dwinmsgs)
#
# # partial block on :x2 should return false
# cliq = whichCliq(tree, :x2)
# prnt = getParent(tree, cliq)[1]
# dwinmsgs = prepCliqInitMsgsDown!(fg, tree, prnt)
# getCliqSiblingsPartialNeeds(tree, cliq, prnt, dwinmsgs)
#
#
# # after :x2 finishes, partial block on :x4 should return false


idx = whichCliq(tree, :x1).index
stf_x1 = IIF.tryCliqStateMachineSolve!(fg, tree, idx, cliqHistories, drawtree=true, N=100, recordcliqs=fsy)
# t_x1 = @async



drawPosesLandms(fg)
