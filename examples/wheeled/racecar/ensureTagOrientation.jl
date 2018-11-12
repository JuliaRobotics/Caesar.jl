# script to confirm all AprilTag sightings are consistent

using YAML, JLD, HDF5
using AprilTags
using Images, ImageView, ImageDraw
using CoordinateTransformations, Rotations, StaticArrays
using MeshCat
using GeometryTypes

# temporarily using Python / OpenCV for solvePnP
using PyCall
@pyimport numpy as np
@pyimport cv2

# load utility function for racecar
include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","racecarUtils.jl"))
include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","cameraUtils.jl"))
include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","visualizationUtils.jl"))

# load camera calibration
cfg = loadConfig()
cw, ch = cfg[:intrinsics][:cx], cfg[:intrinsics][:cy]
fx = fy = cfg[:intrinsics][:fx]
camK = [fx 0 cw; 0 fy ch; 0 0 1]
tagsize = 0.172
# k1,k2 = cfg[:intrinsics][:k1], cfg[:intrinsics][:k2] # makes it worse
k1, k2 = 0.0, 0.0

# tag extrinsic rotation
Rx = RotX(-pi/2)
Rz = RotZ(-pi/2)
bTc= LinearMap(Rz) ∘ LinearMap(Rx)

# image loading directory
datafolder = ENV["HOME"]*"/data/racecar/straightrun3/"
imgfolder = "images"

# Figure export folder
currdirtime = now()
imgdir = joinpath(ENV["HOME"], "Pictures", "testimgs", "$(currdirtime)")
mkdir(imgdir)
mkdir(imgdir*"/tags")

# process images
camlookup = prepCamLookup(175:5:370)
IMGS, TAGS = detectTagsViaCamLookup(camlookup, datafolder*imgfolder, imgdir)
tag_bag = prepTagBag(TAGS)

# save the tag bag file for future use
@save imgdir*"/tag_det_per_pose.jld" tag_bag




# 3D visualization
vis = Visualizer()
open(vis)

setobject!(vis["home"], Triad(1.0))




currtag = ""
bTt = LinearMap(Translation(0.0,0,0) ∘ Quat(1.0,0,0,0))


psid = 5
tags = tag_bag[psid]

# tidx = 1
# tag = tags[tidx]
for (tidx, tag) in tags

currtag = "tag$(tidx)"
Q = Quat(tag[:quat]...)
cTt = (Translation(tag[:pos]...) ∘ LinearMap(Q))
bTt = bTc ∘ cTt
drawTagDetection(vis, currtag, Q, tag[:pos], bTc, tag[:bP2t], posename=Symbol("x$psid"))

end




#
