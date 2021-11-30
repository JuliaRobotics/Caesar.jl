### -- Calibration Example --
### This example shows how to quickly use an optimizer and AprilTags
### to calculate the best-fit extrinsic camera parameters.
# TODO:
# 1. Use all the tags in the optimizer to calculate the error
# 2. Use multiple cameras/images to improve result
# 3. Build out the optimizer for intrinsic as well as extrinsic parameters
# 4. Show tracking using factor graph

using AprilTags
using Images
using CoordinateTransformations, Rotations
using Plots
using ImageTransformations, LinearAlgebra
using ImageProjectiveGeometry

##
cd(@__DIR__())
# img = load("20211102_144705.jpg");
img = load("calibration_board_small.jpg");

## Inputs
# It's imporant that you measure and specify the tag length correctly here
# 30 mm is just a guess, insert your own correct tag measurements here.
taglength = 38.1 # millimeters
disc_radius = 1000 + taglength/2.0 # Radius of the center of tags in mm 
tag_sequence = collect(1:8)

## Detect the tags
detector = AprilTagDetector()
tags = detector(img) .|> deepcopy
if length(tags) != length(tag_sequence)
  @warn "There are $(length(tags)) in the image but expecting $(length(tag_sequence)), missing tag IDs: $(symdiff([t.id for t in tags], tag_sequence))"
end
# Free the detector memory
freeDetector!(detector)

## Get the coordinates of the tag centers
# Tag 1 will be the origin (can pick anything but this is easiest)
# and we can offset anything by that
θs = [(t-1)*π/4.0-pi/2.0 for t in tag_sequence] # Cartesian locations for each tag
# actual_tag_centers = [
#   CartesianFromPolar()(Polar(radius, ang))+[0;radius] for ang in θs]
actual_tag_centers = [
  CartesianFromPolar()(Polar(disc_radius, ang)) for ang in θs]
  
# Points in tag space
cP = Matrix{Float64}(undef,3,0)
tP = Matrix{Float64}(undef,3,0)
for t in tags
  cP = hcat(cP, [t.c;1])
  tP = hcat(tP, [actual_tag_centers[t.id];1])
end

tHc = homography2d(tP,cP)

##

H = LinearMap(tHc)
push1(x) = [x; 1] # convinience function
s = 1
scale = LinearMap(s*I) # scaling transforms
iscale = LinearMap(I/s) 
itform = PerspectiveMap() ∘ H ∘ push1 ∘ iscale # add the scaling to the transformation pipelines
tform =  scale ∘ PerspectiveMap() ∘ inv(H) ∘ push1
imgw = warp(img', itform, ImageTransformations.autorange(img, tform))' # nice, the image size more useful
imgw = warp(img', itform, (-1100:1100, -1100:1100))' # nice, the image size more useful

imgw
