## BEFORE RUNNING THIS SCRIPT, MAKE SURE ros is in the environment

# source /opt/ros/melodic/setup.bash


## Project path variables

projdir = @__DIR__


## Prepare python version

using Pkg

# ENV["PYTHON"] = "/usr/bin/python3.6"
ENV["PYTHON"] = "/usr/bin/python"
Pkg.build("PyCall")
using PyCall

## Load rosbag reader

include( joinpath(projdir,"..","Utils","RosbagSubscriber.jl") )


## prepare ROS integration

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

using Caesar, RoME, IncrementalInference
using DistributedFactorGraphs
using TransformUtils

## add 3D vis

using MeshCat, GeometryTypes, ColorTypes, CoordinateTransformations

vis = Visualizer()
open(vis)

## Constant parameters

# using JSON2
const ImgType = Array{ColorTypes.RGB{FixedPointNumbers.Normed{UInt8,8}},2}

# Consume rosbag with car data
bagfile = joinpath(ENV["HOME"],"data/racecar/mrg/lab_run.bag")
leftimgtopic = "/zed/zed_node/left/image_rect_color/compressed"
# leftimgtopic = "/zed/zed_node/left_raw/image_raw_color/compressed"
rightimgtopic = "/zed/zed_node/right/image_rect_color/compressed"

zedodomtopic = "/zed/zed_node/odom"


# from bagfile
fx = 340.59393310546875
fy = 340.59393310546875
cx = 330.41748046875
cy = 196.3251953125

K = [-fx 0  cx;
      0 fy cy]

WEIRDOFFSET = Dict(:right => 4267, :zedodo => 3073)

gui = imshow_gui((600, 100), (1, 2))  # 2 columns, 1 row of images (each initially 300×300)
canvases = gui["canvas"]

detector = AprilTagDetector()

##

include(joinpath(@__DIR__, "CarSlamUtils.jl"))

## start solver

defaultFixedLagOnTree!(slam.dfg, 30, limitfixeddown=true)
getSolverParams(slam.dfg).dbg = true
ST = manageSolveTree!(slam.dfg, slam.solveSettings, dbg=true)



##

loop!(BagSubscriber)
loop!(BagSubscriber)
loop!(BagSubscriber)

##

sleep(0.01)  # allow gui sime time to setup
for i in 1:5000
  loop!(BagSubscriber)
  if 2 <= length(slam.solveSettings.poseSolveToken.data)
    @info "delay for solver, canTakePoses=$(slam.solveSettings.canTakePoses), tokens=$(slam.solveSettings.poseSolveToken.data)"
    sleep(0.5)
  end
end




## discovery



tree, smt, hist = solveTree!(slam.dfg)

using RoMEPlotting, Gadfly
Gadfly.set_default_plot_size(35cm, 25cm)

# drawPoses(slam.dfg, spscale=0.3, drawhist=false)
drawPosesLandms(slam.dfg, spscale=0.3, drawhist=false)

reportFactors(slam.dfg, Pose2Pose2, show=false)


#
