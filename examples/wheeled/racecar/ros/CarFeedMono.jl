## BEFORE RUNNING THIS SCRIPT, MAKE SURE ros is in the environment

# source /opt/ros/melodic/setup.bash

##

using Distributed


## Project path variables

projdir = @__DIR__


## Prepare python version

using Pkg

# ENV["PYTHON"] = "/usr/bin/python3.6"
ENV["PYTHON"] = "/usr/bin/python"
Pkg.build("PyCall")
using PyCall


## prepare ROS integration

# this will trigger inclusion of RobotOS functionality in Caesar too.
using RobotOS

# standard types
@rosimport sensor_msgs.msg: CompressedImage
@rosimport geometry_msgs.msg: PoseStamped

rostypegen()

### Rest of Julia types

using LinearAlgebra
using ImageMagick
using ImageView
using ImageFeatures, ImageDraw, Images, CoordinateTransformations
using DataStructures
using ColorTypes, FixedPointNumbers
using AprilTags

using Gtk.ShortNames

using Caesar
@everywhere using Caesar
@everywhere using RoME, IncrementalInference, DistributedFactorGraphs, TransformUtils


## add 3D vis

using MeshCat, GeometryTypes, ColorTypes, CoordinateTransformations

vis = Visualizer()
open(vis)

## Constant parameters

# using JSON2
const ImgType = Array{ColorTypes.RGB{FixedPointNumbers.Normed{UInt8,8}},2}

# Consume rosbag with car data
bagfile = joinpath(ENV["HOME"],"data/racecar/labrun5/labRun5.bag")

leftimgtopic = "/zed/left/image_rect_color"
# rightimgtopic = "/zed/right/image_rect_color"
zedodomtopic = "/zed/odom"


# from bagfile
fx = 341.4563903808594
fy = 341.4563903808594
cx = 329.19091796875
cy = 196.3658447265625

K = [-fx 0  cx;
      0 fy cy]


##

include(joinpath(@__DIR__, "CarSlamUtilsMono.jl"))

WEIRDOFFSET = Dict{Symbol, Int}() # Dict(:camOdo => 3073)

gui = imshow_gui((600, 100), (1, 1))  # 2 columns, 1 row of images (each initially 300×300)
canvases = gui["canvas"]
detector = AprilTagDetector()

tools = RacecarTools(detector)


slam = SLAMWrapperLocal()
getSolverParams(slam.dfg).drawtree = true
getSolverParams(slam.dfg).showtree = false

addVariable!(slam.dfg, :x0, Pose2)
addFactor!(slam.dfg, [:x0], PriorPose2(MvNormal(zeros(3),diagm([0.1,0.1,0.01].^2))))

bagSubscriber = RosbagSubscriber(bagfile)

syncz = SynchronizeCarMono(syncList=[:leftFwdCam;:camOdo])
fec = FrontEndContainer(30,slam,bagSubscriber,syncz,tools)

bagSubscriber(leftimgtopic, leftImgHdlr, fec)
bagSubscriber(zedodomtopic, odomHdlr, fec, WEIRDOFFSET)



## start solver

defaultFixedLagOnTree!(slam.dfg, 30, limitfixeddown=true)
# getSolverParams(slam.dfg).dbg = true
getSolverParams(slam.dfg).drawtree = true
getSolverParams(slam.dfg).showtree = false
ST = manageSolveTree!(slam.dfg, slam.solveSettings, dbg=true)



##

loop!(bagSubscriber)
loop!(bagSubscriber)
loop!(bagSubscriber)

##

getSolverParams(slam.dfg).drawtree = false

sleep(0.01)  # allow gui sime time to setup
while loop!(bagSubscriber)
# for i in 1:5000
  # loop!(bagSubscriber)
  blockProgress(slam) # required to prevent duplicate solves occuring at the same time
end

## close all


stopManageSolveTree!(slam)

delete!(vis)


## discovery



# tree, smt, hist = solveTree!(slam.dfg)

using RoMEPlotting, Gadfly
Gadfly.set_default_plot_size(35cm, 20cm)

# drawPoses(slam.dfg, spscale=0.3, drawhist=false)
pl = drawPosesLandms(slam.dfg, spscale=0.3, drawhist=false)
pl |> PDF(joinLogPath(slam.dfg,"fg_$(slam.poseCount).pdf"))
# drawPosesLandms(fg4, spscale=0.3, drawhist=false)

pl = reportFactors(slam.dfg, Pose2Pose2, show=false)
# pl |> PDF(joinLogPath(slam.dfg,"fg_$(slam.poseCount).pdf"))
# reportFactors(fg4, Pose2Pose2, show=false)



#
