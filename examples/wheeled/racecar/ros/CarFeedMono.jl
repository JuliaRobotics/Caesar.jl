## BEFORE RUNNING THIS SCRIPT, MAKE SURE ros is in the environment

## Get user commands

# Populates `parsed_args`
include(joinpath(dirname(@__DIR__),"parsecommands.jl"))

# assume in Atom editor (not scripted use)
if length(ARGS) == 0
  parsed_args["folder_name"] = "labrun8"
  parsed_args["remoteprocs"] = 10
  parsed_args["vis2d"] = true
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

# bring up the multiprocess cluster
include(joinpath(@__DIR__, "CarFeedCommonSetup.jl"))

# a few required utils
include(joinpath(@__DIR__, "CarSlamUtilsMono.jl"))


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

dfg_datafolder = getLogPath(slam.dfg)
datastore = FileDataStore("$dfg_datafolder/bigdata")

# also store parsed_args used in this case
fid = open(joinLogPath(slam.dfg,"args.json"),"w")
println(fid, JSON2.write(parsed_args))
close(fid)

# TODO add to results.log
fid = open(joinpath(dirname(getLogPath(slam.dfg)),"results.log"),"a")
resdirname = splitpath(getLogPath(slam.dfg))[end]
thisfile = splitpath(@__FILE__)[end]
println(fid, "$resdirname -- $thisfile, $(parsed_args["folder_name"]), $ARGS")
close(fid)


bagSubscriber = RosbagSubscriber(bagfile)

syncz = SynchronizeCarMono(30,syncList=[:leftFwdCam;:camOdo])
fec = FrontEndContainer(slam,bagSubscriber,syncz,tools,datastore)

# callbacks via Python
bagSubscriber(leftimgtopic, leftImgHdlr, fec)
bagSubscriber(zedodomtopic, odomHdlr, fec, WEIRDOFFSET)
bagSubscriber(joysticktopic, joystickHdlr, fec)


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


##


sleep(0.01)  # allow gui sime time to setup
while loop!(bagSubscriber)
# for i in 1:2000
#   loop!(bagSubscriber)
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

tree = solveTree!(fec.slam.dfg)

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
