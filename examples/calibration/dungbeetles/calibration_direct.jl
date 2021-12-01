### -- Calibration Example --
### This example shows how to quickly use an optimizer and AprilTags
### to calculate the best-fit extrinsic camera parameters.
# TODO:
# 1. Use all the tags in the optimizer to calculate the error
# 2. Use multiple cameras/images to improve result
# 3. Build out the optimizer for intrinsic as well as extrinsic parameters
# 4. Show tracking using factor graph

using FreeTypeAbstraction, AprilTags
using LinearAlgebra
using Images
using ImageView
using Optim
using CoordinateTransformations, Rotations
using Plots

##

cd( @__DIR__ )
# img = load("20211102_144705.jpg");
img = load("calibration_board_small.jpg");

## Inputs
# It's imporant that you measure and specify the tag length correctly here
# 30 mm is just a guess, insert your own correct tag measurements here.
taglength = 0.0381 # millimeters
disc_radius = 1 + taglength/2.0 # Radius of the center of tags in meters 
tag_sequence = collect(1:8)

# Get the coordinates of the tag centers
# Tag 1 will be the origin (can pick anything but this is easiest)
# and we can offset anything by that
radius = disc_radius/(taglength) # of the circle drawn on the arena + half the size of the printed apriltag
# Cartesian locations for each tag
# using 0deg as x-axis, and +90deg as y-axis  # [5.0; 50; 95; 140; 185; 230; 275; 320;]
θs = range(0,360,length=9)[1:end-1] |> collect |> x->x.+5 .|> deg2rad 
# reproject onto [-pi,pi)
θs = rem2pi.(θs, RoundNearest)    
# θs = [(t-1)*π/4.0-pi/2.0 for t in tag_sequence] # not quie right, has some angles bigger than [-pi,pi)

# format cen=[i,j] <==> img[i,j] <==> [down,right]
dest_center = [] # [0;0] # [0;radius]
actual_tag_centers = [
  CartesianFromPolar()(Polar(radius, ang)) for ang in θs]



## Detect the tags
detector = AprilTagDetector()
tags = detector(img) .|> deepcopy
if length(tags) != length(tag_sequence)
  @warn "There are $(length(tags)) in the image but expecting $(length(tag_sequence)), missing tag IDs: $(symdiff([t.id for t in tags], tag_sequence))"
end
# Free the detector memory
freeDetector!(detector)


## draw the detections
# using FreeTypeAbstraction
# `img[i,j]` implies `width == x == j`, and `height == y == i`, top left corner `(0,0)`
# just a rough guess of intrinsics (not really required here yet)
fx = size(img, 2)
fy = fx
cx = size(img, 1)/2
cy = size(img, 2)/2

K = [fx 0  cx;
      0 fy cy] # intrinsics
imgd = drawTags(img, K, tags);
# imshow(imgd)


## draw desired tag locations for validation

# Points in tag space
_sax = [2;1] # flip axes used in AprilTags
cP = Matrix{Float64}(undef,3,0)
tP = Matrix{Float64}(undef,3,0)
for t in tags
  # use convention [i,j,1] <==> img[i,j] <==> [down,right]
  cP = hcat(cP, [t.c[_sax];1])
  tP = hcat(tP, [actual_tag_centers[t.id][_sax];1])
end

# Calculating the homography matrix (camera to tag) directly
tHc = (tP * (cP')) * inv(cP * (cP'))

## Plot it
## Plotting functions
# Plotting so we can can see the actual vs. measured
function plotActual(actual_tag_centers::Vector)
  scatter(
    [c[1] for c in actual_tag_centers], 
    [c[2] for c in actual_tag_centers],
    title = "Tag centers in tag space (normalized by tag length)",
    xlabel = "X, img[i,.]",
    ylabel = "Y, img[.,j]",
    m = (0.5, :hex, 12),
    bg = RGB(0.2, 0.2, 0.2),
    label = "Actual centers", 
    series_annotations = [string(k) for (k,c) in enumerate(actual_tag_centers)])
end
function plotMeasured(tags::Vector{AprilTag}, base_tag_id::Int, tHc::Matrix)
  tag = tags[base_tag_id]
  measured_tag_centers = Dict(
    [t.id for t in tags] .=> 
    [(tHc * [t.c[_sax]; 1]) for t in tags])  #? Why is there residual in Z? Keep an eye on it in the optimization.
  scatter!(
    [c[1] for (k,c) in measured_tag_centers], 
    [c[2] for (k,c) in measured_tag_centers],
    m = (0.5, :dtriangle, 12),
    bg = RGB(0.2, 0.2, 0.2),
    label = "Measured (origin is $(tag.id))",
    series_annotations = [string(k) for (k,c) in measured_tag_centers])
end  
plotActual(actual_tag_centers)
plotMeasured(tags, 1, tHc)

## WIP: Plotting the rectified 
# Now, multiplying the transformed value by the tag length will give you real-world coordinates

using ImageTransformations
H = LinearMap(tHc)
push1(x) = [x; 1] # convinience function
s = 1
scale = LinearMap(s*I) # scaling transforms
iscale = LinearMap(I/s) 
itform = PerspectiveMap() ∘ H ∘ push1 ∘ iscale # add the scaling to the transformation pipelines
tform =  scale ∘ PerspectiveMap() ∘ inv(H) ∘ push1
imgw = warp(img', itform, ImageTransformations.autorange(img, tform)) # nice, the image size more useful
imshow(imgw) # but the rectification is wrong!?