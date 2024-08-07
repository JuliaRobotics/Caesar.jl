## BEFORE RUNNING THIS SCRIPT, MAKE SURE ros is in the environment

cd(ENV["HOME"]*"/.julia/dev/Caesar/examples/wheeled/racecar/ros")

## Get user commands

# Populates `parsed_args`
include(joinpath(dirname(@__DIR__),"parsecommands.jl"))

# assume in Atom editor (not scripted use)
if length(ARGS) == 0
  @info "Using default commands"
  parsed_args["folder_name"] = "labrun2"
  parsed_args["remoteprocs"] = 0
  parsed_args["localprocs"] = 4
  parsed_args["vis2d"] = true
  parsed_args["vis3d"] = false
  parsed_args["imshow"] = true
  parsed_args["msgloops"] = 5000
  parsed_args["usesimmodels"] = true
  parsed_args["dbg"] = true
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


## load neural models

include(joinpath(dirname(@__DIR__), "LoadPyNNTxt.jl"))

allModels = []
for i in 0:99
# /home/dehann/data/racecar/results/conductor/models/retrained_network_weights0
  if parsed_args["usesimmodels"]
    push!(allModels, loadPose2OdoNNModelIntoFlux(ENV["HOME"]*"/data/racecar/results/conductor/sim_models/sim_network_weights$i", pad=true) )
  else
    push!(allModels, loadPose2OdoNNModelIntoFlux(ENV["HOME"]*"/data/racecar/results/conductor/models/retrained_network_weights$i", pad=true) )
  end
end

@assert length(allModels) == 100 "Flux models did not load correctly"


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
getSolverParams(slam.dfg).useMsgLikelihoods = true
getSolverParams(slam.dfg).dbg = parsed_args["dbg"]

mkpath(getLogPath(slam.dfg))

# datastore = FolderStore( :default, joinLogPath(slam.dfg,"data") )
datastore = FolderStore{Vector{UInt8}}(:default_folder_store, joinLogPath(slam.dfg,"data"))
mkpath(joinLogPath(slam.dfg,"data"))
addBlobStore!(slam.dfg, datastore)

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

syncz = SynchronizeCarMono(200,syncList=[:leftFwdCam;:camOdo])
fec = FrontEndContainer(slam,bagSubscriber,syncz,tools,datastore)

# callbacks via Python
bagSubscriber(leftimgtopic, leftImgHdlr, fec)
bagSubscriber(zedodomtopic, odomHdlr, fec, WEIRDOFFSET, allModels ) # extra value triggers flux mode
bagSubscriber(joysticktopic, joystickHdlr, fec)
# (x)->JlOdoPredictorPoint2(x, allModels)

defaultFixedLagOnTree!(slam.dfg, 50, limitfixeddown=true)
getSolverParams(slam.dfg).dbg = false
# getSolverParams(slam.dfg).drawtree = true
# getSolverParams(slam.dfg).showtree = false


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

# must wait here till loop! finishes

## store the camera calibration in the first pose

camCalib = Dict(:size=>size(K), :vecK=>vec(K))
addData!(slam.dfg,:default_folder_store,:x0,:camCalib,
         Vector{UInt8}(JSON2.write(camCalib)), mimeType="application/json/octet-stream", 
         description="reshape(camCalib[:vecK], camCalib[:size]...)") 



## start solver


ST = manageSolveTree!(slam.dfg, slam.solveSettings, dbg=false)

# getSolverParams(slam.dfg).dbg = true

## Run main ROS listener loop
if parsed_args["imshow"]
  Gtk.showall(gui["window"])
  sleep(0.1)  # allow gui some time to setup
end

rosloops = 0
let rosloops = rosloops
while loop!(bagSubscriber)
  # plumbing to limit the number of messages
  rosloops += 1
  if parsed_args["msgloops"] < rosloops
    @warn "reached --msgloops limit of $rosloops"
    break
  end
  # delay progress for whatever reason
  # @show "blcok progress"
  blockProgress(slam) # required to prevent duplicate solves occuring at the same time
end
end

## close all


stopManageSolveTree!(slam)

parsed_args["vis3d"] ? delete!(vis) : nothing

# wait for previous and then ensure all data is solved at least once
@info "CarFeedMono.jl is waiting on penultimate trigger solve to finalize."
blockSolvingInProgress(fec.slam)
checkSolveStrideTrigger!(fec.slam, force=true)
sleep(10)
@info "CarFeedMono.jl is waiting on last trigger solve to finalize."
blockSolvingInProgress(fec.slam)


# save final graph
saveDFG(slam.dfg, joinLogPath(slam.dfg,"finalGraph"))

# generate the results
parsed_args["previous"] = getLogPath(slam.dfg)
include(joinpath(@__DIR__, "CarSlamGenResults.jl"))

#


# fg = loadDFG("fg_final_resolve.tar.gz")
# getSolverParams(fg).logpath = pwd()
# datastore = FolderStore{Vector{UInt8}}(:default_folder_store, joinLogPath(fg,"data"))
# addBlobStore!(fg, datastore)


#