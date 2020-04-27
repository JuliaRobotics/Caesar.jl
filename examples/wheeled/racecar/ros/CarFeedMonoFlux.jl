## BEFORE RUNNING THIS SCRIPT, MAKE SURE ros is in the environment

## Get user commands

# Populates `parsed_args`
include(joinpath(dirname(@__DIR__),"parsecommands.jl"))

# assume in Atom editor (not scripted use)
if length(ARGS) == 0
  parsed_args["folder_name"] = "labrun8"
  parsed_args["remoteprocs"] = 0
  parsed_args["localprocs"] = 2
  parsed_args["vis2d"] = true
  parsed_args["vis3d"] = false
  parsed_args["imshow"] = true
end


## load main process processes only

if parsed_args["vis2d"]
  using Cairo
  using Gadfly
  using RoMEPlotting
  Gadfly.set_default_plot_size(35cm, 20cm)
end

# create context for vis3d
vis = if parsed_args["vis3d"]
  using MeshCat, GeometryTypes, ColorTypes, CoordinateTransformations
  vis = Visualizer()
  open(vis)
  vis
end

## bring in all the required source code

using CuArrays
using Flux

# bring up the multiprocess cluster
include(joinpath(@__DIR__, "CarFeedCommonSetup.jl"))

# a few required utils
include(joinpath(@__DIR__, "CarSlamUtilsMono.jl"))


## load neural models

include(joinpath(dirname(@__DIR__), "LoadPyNNTxt.jl"))

allModels = []
for i in 0:99
# /home/dehann/data/racecar/results/conductor/models/retrained_network_weights0
  push!(allModels, loadTfModelIntoFlux(ENV["HOME"]*"/data/racecar/results/conductor/models/retrained_network_weights$i") )
end


##

gui, canvases = if parsed_args["imshow"]
  gui = imshow_gui((1000, 200), (1, 2))  # 2 columns, 1 row of images (each initially 300×300)
  canvases = gui["canvas"]
  # allow gui time to load
  sleep(1.0)
  gui, canvases
else
  nothing, nothing
end

tools = RacecarTools(detector)

## SLAM portion

slam = SLAMWrapperLocal()
getSolverParams(slam.dfg).drawtree = true
getSolverParams(slam.dfg).showtree = false

datastore = FileDataStore(joinLogPath(slam.dfg,"bigdata"))

# also store parsed_args used in this case
fid = open(joinLogPath(slam.dfg,"args.json"),"w")
println(fid, JSON2.write(parsed_args))
close(fid)

# TODO add to results.log
fid = open(joinpath(dirname(getLogPath(slam.dfg)),"results.log"),"a")
resdirname = splitpath(getLogPath(slam.dfg))[end]
println(fid, "$resdirname -- CarFeedMono.jl, $(parsed_args["folder_name"]), $ARGS")
close(fid)


bagSubscriber = RosbagSubscriber(bagfile)

syncz = SynchronizeCarMono(200,syncList=[:leftFwdCam;:camOdo])
fec = FrontEndContainer(slam,bagSubscriber,syncz,tools,datastore)

# callbacks via Python
bagSubscriber(leftimgtopic, leftImgHdlr, fec)
bagSubscriber(zedodomtopic, odomHdlr, fec, WEIRDOFFSET, predFluxOdo2, allModels ) # extra value triggers flux mode
bagSubscriber(joysticktopic, joystickHdlr, fec)
# (x)->JlOdoPredictorPoint2(x, allModels)

defaultFixedLagOnTree!(slam.dfg, 50, limitfixeddown=true)
# getSolverParams(slam.dfg).dbg = true
getSolverParams(slam.dfg).drawtree = true
getSolverParams(slam.dfg).showtree = false


##

# (fec.synchronizer.leftFwdCam |> last)[2] |> collect |> imshow

# consume a few messages to find initialization timestamp
for i in 1:100
  @show i, bagSubscriber.nextMsgChl
  0 == length(fec.synchronizer.leftFwdCam) || 0 == length(fec.synchronizer.camOdo) ? nothing : break
  loop!(bagSubscriber)
end

## initialize the factor graph

T0 = nanosecond2datetime(fec.synchronizer.leftFwdCam[1][2])

addVariable!(slam.dfg, :x0, Pose2, timestamp=T0)
addFactor!(slam.dfg, [:x0], PriorPose2(MvNormal(zeros(3),diagm([0.1,0.1,0.01].^2))), timestamp=T0)


## start solver


ST = manageSolveTree!(slam.dfg, slam.solveSettings, dbg=false)



##  Special async task to add Neural odo to fg when data becomes available.

# # latest pose
# allVars = ls(fec.slam.dfg, r"x\d") |> sortDFG
# # does it have FluxModelsPose2Pose2 factor?
# allFluxFct = ls(fec.slam.dfg, FluxModelsPose2Pose2)
# mask = (x->intersect(ls(fec.slam.dfg, x), allFluxFct ) |> length).(allVars) .== 0
# varsWithout = allVars[mask][end-5:end]
#
# # what are the command values from previous pose
# let prevPs = prevPs, i=i
# prevPs = varsWithout[1]
# i = 2
# for i in 2:length(varsWithout)
#
# ps = varsWithout[i]
# theVar = getVariable(fec.slam.dfg, ps)
# # skip if no entry yet
# # hasDataEntry(theVar, :JOYSTICK_CMD_VALS) ? nothing : continue
# cmdData = fetchDataElement(theVar, fec.datastore, :JOYSTICK_CMD_VALS)
# throttle = (x->x[3].axis[2]).(cmdData)
# steering = (x->x[3].axis[4]).(cmdData)
# DT = ( getTimestamp(theVar) - getTimestamp(getVariable(fec.slam.dfg, prevPs)) ).value*1e-3
# xj = getPPE(fec.slam.dfg, ps).suggested
# xi = getPPE(fec.slam.dfg, prevPs).suggested
# biRw = TU.R(-xi[3])
# biV = biRw * (xj[1:2] - xi[1:2]) / DT
# biVrep = repeat(biV', length(throttle))
# joyval = hcat(throttle, steering, biVrep)
#
# #
# @show prevPs, ps, size(joyval,1)
#
# joyval
# itpJoy = interpToOutx4(joyval)
#
# JlOdoPredictorPoint2(itpJoy,allModels)
#
#
# prevPs = ps
# end
# end
#
# # interpolate joyvals to right size
# # joyval
#
# # the naive model (should be the camera)
# DXmvn = MvNormal(zeros(3),diagm([0.4;0.1;0.4].^2))
#
# fmp2 = FluxModelsPose2Pose2(x->JlOdoPredictorPoint2(x,allModels), joyval25, DXmvn,0.6)
#
#
# # convert to [25 x 4] input



##

sleep(0.01)  # allow gui some time to setup
# while loop!(bagSubscriber)
for i in 1:1000
  loop!(bagSubscriber)
  blockProgress(slam) # required to prevent duplicate solves occuring at the same time
end

## close all


stopManageSolveTree!(slam)

parsed_args["vis3d"] ? delete!(vis) : nothing

# wiat for previous and then ensure all data is solved at least once
@info "CarFeedMono.jl is waiting on penultimate trigger solve to finalize."
blockSolvingInProgress(fec.slam)
checkSolveStrideTrigger!(fec.slam, force=true)
sleep(10)
@info "CarFeedMono.jl is waiting on last trigger solve to finalize."
blockSolvingInProgress(fec.slam)


## interpose results

allD = jsonResultsSLAM2D(fec)

allStr = JSON2.write(allD)

fid = open(joinLogPath(fec.slam.dfg, "$(runfile)_results.json"),"w")
println(fid, allStr)
close(fid)

## save the factor graph

saveDFG(fec.slam.dfg, joinLogPath(fec.slam.dfg, "fg_$(slam.poseCount)"))


## draw results

if parsed_args["vis2d"]

# drawPoses(slam.dfg, spscale=0.3, drawhist=false)
pl = drawPosesLandms(fec.slam.dfg, spscale=0.3, drawhist=false)
pl |> PDF(joinLogPath(fec.slam.dfg,"fg_$(slam.poseCount).pdf"))
# drawPosesLandms(fg4, spscale=0.3, drawhist=false)

if parsed_args["report_factors"]
  reportFactors(fec.slam.dfg, Pose2Pose2, show=false)
end

end

## batch resolve

if parsed_args["batch_resolve"]

# fg2 = deepcopy(fec.slam.dfg)

dontMarginalizeVariablesAll!(fec.slam.dfg)
foreach(x->setSolvable!(fec.slam.dfg, x, 1), ls(fec.slam.dfg))
foreach(x->setSolvable!(fec.slam.dfg, x, 1), lsf(fec.slam.dfg))

tree, smt, hist = solveTree!(fec.slam.dfg)

saveDFG(fec.slam.dfg, joinLogPath(fec.slam.dfg, "fg_$(slam.poseCount)_resolve"))

end

## after resolve interpose results

allD = jsonResultsSLAM2D(fec)

allStr = JSON2.write(allD)

fid = open(joinLogPath(fec.slam.dfg, "$(runfile)_results_batchresolve.json"),"w")
println(fid, allStr)
close(fid)


## draw a second time


if parsed_args["batch_resolve"] && parsed_args["vis2d"]


pl = drawPosesLandms(fec.slam.dfg, spscale=0.3, drawhist=false)
pl |> PDF(joinLogPath(slam.dfg,"fg_$(slam.poseCount)_resolve.pdf"))

end



#
