## BEFORE RUNNING THIS SCRIPT, MAKE SURE ros is in the environment

if parsed_args["imshow"]
  using ImageView
  using Gtk.ShortNames
end

using LinearAlgebra
using ImageMagick
using ImageFeatures, ImageDraw, Images, CoordinateTransformations
using DataStructures
using ColorTypes, FixedPointNumbers
using FreeTypeAbstraction # for drawTagID!
using AprilTags
using JSON2
using Dates
using DataInterpolations

using CuArrays
using Flux
using RoME
using Caesar


# source /opt/ros/melodic/setup.bash

## Project path variables

projdir = @__DIR__


## work with multiple processes

using Distributed

# populate remote machines on cluster later (was need for Python workaround and left)
prcs115 = Int[]

# ## Process on this machine but not remote machines
# using Caesar

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

addprocs(parsed_args["localprocs"])


## Start processes on remote machines

if 0 < parsed_args["remoteprocs"]
  machines = []
  allmc = split(ENV[parsed_args["remoteserver"]], ';')
  for mc in allmc
    push!(machines, (mc,parsed_args["remoteprocs"]))
  end
  prcs115 = addprocs(machines)
end

## Load solver libraries everywhere

using CuArrays, Flux
using RoME
using DataInterpolations
@everywhere using CuArrays, Flux
@everywhere using RoME
@everywhere using IncrementalInference, DistributedFactorGraphs, TransformUtils, DataInterpolations


## Constant parameters

# using JSON2
const ImgType = Array{ColorTypes.RGB{FixedPointNumbers.Normed{UInt8,8}},2}

# Consume rosbag with car data
runfile = parsed_args["folder_name"] # "labrun7"
bagfile = joinpath(ENV["HOME"],"data/racecar/$runfile.bag")

leftimgtopic = "/zed/left/image_rect_color"
# rightimgtopic = "/zed/right/image_rect_color"
zedodomtopic = "/zed/odom"
joysticktopic = "/joy"


# from bagfile -- TODO -- make this simpler
fx = 341.4563903808594
fy = 341.4563903808594
cx = 329.19091796875
cy = 196.3658447265625

K = [-fx 0  cx;
      0 fy cy]

# guessing
# 758016/3 = 252672
# (c, a / c) = (672, 376.0)


WEIRDOFFSET = Dict{Symbol, Int}() #:cmdVal => -11345)
detector = AprilTagDetector()


## load visualization if requested

if parsed_args["vis2d"]

using Cairo
using Gadfly
using RoMEPlotting
Gadfly.set_default_plot_size(35cm, 20cm)

end

#
