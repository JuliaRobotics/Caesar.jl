## BEFORE RUNNING THIS SCRIPT, MAKE SURE ros is in the environment

# source /opt/ros/melodic/setup.bash

## Project path variables

projdir = @__DIR__

## load main process processes only

using ImageView
using Gtk.ShortNames

using LinearAlgebra
using ImageMagick
using ImageFeatures, ImageDraw, Images, CoordinateTransformations
using DataStructures
using ColorTypes, FixedPointNumbers
using FreeTypeAbstraction # for drawTagID!
using AprilTags

using Caesar

## add 3D vis

using MeshCat, GeometryTypes, ColorTypes, CoordinateTransformations

vis = Visualizer()
open(vis)

## work with multiple processes

using Distributed

# populate remote machines on cluster later (was need for Python workaround and left)
prcs115 = Int[]

## Process on this machine but not remote machines

using Caesar

## Prepare python version

using Pkg
@everywhere using Pkg

# ENV["PYTHON"] = "/usr/bin/python3.6"
@everywhere begin
  ENV["PYTHON"] = "/usr/bin/python"
  Pkg.build("PyCall")
end

using PyCall
@everywhere using PyCall


# for prc in prcs115
#   @info "Load rospy at worker $prc"
#   fut = Distributed.@spawnat prc begin
#     # import subprocess
#     # rc = subprocess.call("/opt/ros/melodic/setup.bash",shell=True)
#     # """
#     py"""
#     import sys
#     sys.path.insert(0, "/opt/ros/melodic/lib/python2.7/dist-packages")
#     """
#     py"""
#     import rospy
#     """
#   end
#   @show fetch(fut)
# end


## prepare ROS integration

# this will trigger inclusion of RobotOS functionality in Caesar too.
using RobotOS

# standard types
# @rosimport sensor_msgs.msg: CompressedImage
# @rosimport geometry_msgs.msg: PoseStamped
#
# rostypegen()

## start common processes on local machine

addprocs(6)

## Start processes on remote machines

machines = [("labuser@192.168.1.115",15)]
prcs115 = addprocs(machines)


## Load solver libraries everywhere

using RoME
@everywhere using RoME
@everywhere using IncrementalInference, DistributedFactorGraphs, TransformUtils


## Constant parameters

# using JSON2
const ImgType = Array{ColorTypes.RGB{FixedPointNumbers.Normed{UInt8,8}},2}

# Consume rosbag with car data
bagfile = joinpath(ENV["HOME"],"data/racecar/labRun7.bag")

leftimgtopic = "/zed/left/image_rect_color"
# rightimgtopic = "/zed/right/image_rect_color"
zedodomtopic = "/zed/odom"
joysticktopic = "/joy"


# from bagfile
fx = 341.4563903808594
fy = 341.4563903808594
cx = 329.19091796875
cy = 196.3658447265625

K = [-fx 0  cx;
      0 fy cy]

# guessing
# 758016/3 = 252672
# (c, a / c) = (672, 376.0)

##

include(joinpath(@__DIR__, "CarSlamUtilsMono.jl"))

WEIRDOFFSET = Dict{Symbol, Int}(:cmdVal => -11345)

gui = imshow_gui((600, 100), (1, 2))  # 2 columns, 1 row of images (each initially 300×300)
canvases = gui["canvas"]
detector = AprilTagDetector()


tools = RacecarTools(detector)


slam = SLAMWrapperLocal()
getSolverParams(slam.dfg).drawtree = true
getSolverParams(slam.dfg).showtree = false

addVariable!(slam.dfg, :x0, Pose2)
addFactor!(slam.dfg, [:x0], PriorPose2(MvNormal(zeros(3),diagm([0.1,0.1,0.01].^2))))

bagSubscriber = RosbagSubscriber(bagfile)

syncz = SynchronizeCarMono(30,syncList=[:leftFwdCam;:camOdo])
fec = FrontEndContainer(slam,bagSubscriber,syncz,tools)

# callbacks via Python
bagSubscriber(leftimgtopic, leftImgHdlr, fec)
bagSubscriber(zedodomtopic, odomHdlr, fec, WEIRDOFFSET)
bagSubscriber(joysticktopic, joystickHdlr, fec)



## start solver

defaultFixedLagOnTree!(slam.dfg, 50, limitfixeddown=true)
# getSolverParams(slam.dfg).dbg = true
getSolverParams(slam.dfg).drawtree = true
getSolverParams(slam.dfg).showtree = false
ST = manageSolveTree!(slam.dfg, slam.solveSettings, dbg=false)



##

# (fec.synchronizer.leftFwdCam |> last)[2] |> collect |> imshow

loop!(bagSubscriber)
loop!(bagSubscriber)
loop!(bagSubscriber)

##


sleep(0.01)  # allow gui sime time to setup
while loop!(bagSubscriber)
# for i in 1:5000
#   loop!(bagSubscriber)
  blockProgress(slam) # required to prevent duplicate solves occuring at the same time
end

## close all


stopManageSolveTree!(slam)

delete!(vis)


## discovery



# tree, smt, hist = solveTree!(slam.dfg)

using Gadfly
using RoMEPlotting
Gadfly.set_default_plot_size(35cm, 20cm)

# drawPoses(slam.dfg, spscale=0.3, drawhist=false)
pl = drawPosesLandms(slam.dfg, spscale=0.3, drawhist=false)
pl |> PDF(joinLogPath(slam.dfg,"fg_$(slam.poseCount).pdf"))
# drawPosesLandms(fg4, spscale=0.3, drawhist=false)

pl = reportFactors(slam.dfg, Pose2Pose2, show=false)
# pl |> PDF(joinLogPath(slam.dfg,"fg_$(slam.poseCount).pdf"))
# reportFactors(fg4, Pose2Pose2, show=false)


dontMarginalizeVariablesAll!(fec.slam.dfg)
foreach(x->setSolvable!(fec.slam.dfg, x, 1), ls(fec.slam.dfg))
foreach(x->setSolvable!(fec.slam.dfg, x, 1), lsf(fec.slam.dfg))


for i in 1:100
  tree, smt, hist = solveTree!(fec.slam.dfg)
  pl = drawPosesLandms(fec.slam.dfg, spscale=0.3, drawhist=false)
  pl |> PDF(joinLogPath(fec.slam.dfg,"fg_resolve_$(i).pdf"))
end


plotLocalProduct(fec.slam.dfg, :l1, levels=5, dims=[1;2])
plotTreeProductUp(fec.slam.dfg, tree, :l1, levels=5, dims=[1;2])

saveDFG(fec.slam.dfg,joinLogPath(fec.slam.dfg,"fg_final"))

## using syncList to fetch interpose buffered data


idxL,idxO = findSyncLatestIdx(fec.synchronizer, syncList=[:leftFwdCam;:cmdVal], weirdOffset=WEIRDOFFSET)


WEIRDOFFSET

fec.synchronizer




## display tag text

using FreeTypeAbstraction




img = (fec.synchronizer.leftFwdCam |> last)[2] |> collect


tagsL = detector(img)

foreach(x->drawTagID!(img, x),tagsL)

imshow(img)


#
