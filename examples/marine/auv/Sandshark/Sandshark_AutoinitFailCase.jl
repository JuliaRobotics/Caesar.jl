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

const TU = TransformUtils

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
# getSolverParams(fg).async = true

tree, smt, hist = solveTree!(fg, recordcliqs=ls(fg), delaycliqs=[:l1;])


# writeGraphPdf(fg, show=true)

drawPosesLandms(fg)


# printCliqHistorySummary(tree, :l1)

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
