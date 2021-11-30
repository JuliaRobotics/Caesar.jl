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
using ImageView
using Optim
using CoordinateTransformations, Rotations
using Plots

img = load("20211102_144705.jpg");

## Inputs
# It's imporant that you measure and specify the tag length correctly here
# 30 mm is just a guess, insert your own correct tag measurements here.
taglength = 0.0381 # millimeters
disc_radius = 1 + taglength/2.0 # Radius of the center of tags in meters 
tag_sequence = collect(1:8)
# It looks like the tags are rotated, so considering 
# a tag facing outward from the circle as correct, and reorienting
# the other tags according to it. 
measured_tag_rotations = [
  pi, # 1
  pi/2.0, # 2 
  3.0*pi/2.0, # 3 
  3.0*pi/2.0, # TODO: Confirm 4 because we don't detect it
  3.0*pi/2.0, # 5
  0, # 6
  pi/2.0, # 7
  3.0*pi/2.0 # 8
  ]

## Detect the tags
detector = AprilTagDetector()
tags = detector(img) .|> deepcopy
if length(tags) != length(tag_sequence)
  @warn "There are $(length(tags)) in the image but expecting $(length(tag_sequence)), missing tag IDs: $(symdiff([t.id for t in tags], tag_sequence))"
end
# Free the detector memory
freeDetector!(detector)
# Show the result
for t in tags
  drawTagBox!(img, t)
end
imshow(img)


## Get the coordinates of the tag centers
# Tag 1 will be the origin (can pick anything but this is easiest)
# and we can offset anything by that
radius = disc_radius/(taglength) # of the circle drawn on the arena + half the size of the printed apriltag
θs = [(t-1)*π/4.0-pi/2.0 for t in tag_sequence] # Cartesian locations for each tag
actual_tag_centers = [
  CartesianFromPolar()(Polar(radius, ang))+[0;radius] for ang in θs]


## Calculating the error
# This function calculates the expected positions (in tag space) for the 
# other tags, given a base tag and a calculated H matrix. We use this as
# the basis for the optimization. There's a lot of room for improvement here.
function calcTagspaceTagPositions(base_tag::AprilTag, H::Matrix, tags::Vector{AprilTag}, measured_tag_rotations::Vector{Float64}, actual_tag_centers::Vector)
  H_pix2tag = RotZ(-measured_tag_rotations[base_tag.id]) * inv(H)
  # Calculate the tags centers in tagspace for each measured tag
  measured_tag_centers = Dict(
    [t.id for t in tags] .=> 
    [(H_pix2tag * [t.c; 1])[1:2] for t in tags])
  # Calculate this tags error
  stats = Dict{Int, Any}()
  for to in tags
    # @info cur = mod(to.id-index,8)+1
    actual = actual_tag_centers[mod(to.id-base_tag.id,8)+1]
    measured = measured_tag_centers[to.id]
    stats[to.id] = Dict{String, Any}(
      "actual" => actual, 
      "measured" => measured, 
      "error" => actual - measured,
      "error2" => sum((actual - measured).^2)
    )
  end
  return stats
end  

# Quick tests: check that we correctly use the center of whichever tag is used as the base  
tag1 = calcTagspaceTagPositions(tags[1], tags[1].H, tags, measured_tag_rotations, actual_tag_centers)
tag6 = calcTagspaceTagPositions(tags[5], tags[5].H, tags, measured_tag_rotations, actual_tag_centers)
# Just doing this by inspection - the base tag should be around 0.


## Plotting functions
# Plotting so we can can see the actual vs. measured
function plotActual(actual_tag_centers::Vector)
  scatter(
    [c[1] for c in actual_tag_centers], 
    [c[2] for c in actual_tag_centers],
    title = "Tag centers in tag space (normalized by tag length)",
    xlabel = "X",
    ylabel = "Y",
    m = (0.5, :hex, 12),
    label = "Actual centers", 
    bg = RGB(0.2, 0.2, 0.2))
end
function plotMeasured(tags::Vector{AprilTag}, base_tag_id::Int, H::Matrix, measured_tag_rotations::Vector)
  tag = tags[base_tag_id]
  H_pix2tag = RotZ(-measured_tag_rotations[base_tag_id]) * inv(H) 
  measured_tag_centers = Dict(
    [t.id for t in tags] .=> 
    [(H_pix2tag * [t.c; 1]) for t in tags])  #? Why is there residual in Z? Keep an eye on it in the optimization.
  scatter!(
    [c[1] for (k,c) in measured_tag_centers], 
    [c[2] for (k,c) in measured_tag_centers],
    m = (0.5, :dtriangle, 12),
    bg = RGB(0.2, 0.2, 0.2),
    label = "Measured (origin is $(tag.id))",
    series_annotations = [string(k) for (k,c) in measured_tag_centers])
end  


## Optimizing for minimal error
# Setting up an optimizer to minimize the error in the other tags
# Choose a base tag to use as the origin
base_tag_id = 1

# The optimizer only works with vectors, so tHc converts the vector back to a matrix.
tHc(h=[1;0;0;0;1;0;0;0]) = [h[1] h[2] h[3]; h[4] h[5] h[6]; h[7] h[8] 1.0]
# sum residual from all tag projections together
fitness(h) = sum(stats -> stats["error2"], values(calcTagspaceTagPositions(tags[base_tag_id], tHc(h), tags, measured_tag_rotations, actual_tag_centers) ) )

# starting point for gradient descent (a rough guess or H matrix)
H_init = tags[base_tag_id].H
# Converting H_init to a row-major vector (vec is column major)
h0 = collect(vec(transpose(H_init)))

# Optimize for the best H
res = Optim.optimize(fitness, h0) # which algorithm: BDFS() OR NelderMead()
h_star = res.minimizer
# the best joint homography
H_star = tHc(h_star)

@info "Optimal H using tag $(base_tag_id) as base:\r\n$(H_star)"
plotActual(actual_tag_centers)
plotMeasured(tags, base_tag_id, H_star, measured_tag_rotations)

# Now, multiplying the transformed value by the tag length will give you real-world coordinates

using ImageTransformations, LinearAlgebra
H = LinearMap(H_star)
push1(x) = [x; 1] # convinience function
s = 10
scale = LinearMap(s*I) # scaling transforms
iscale = LinearMap(I/s) 
itform = PerspectiveMap() ∘ H ∘ push1 ∘ iscale # add the scaling to the transformation pipelines
tform =  scale ∘ PerspectiveMap() ∘ inv(H) ∘ push1
imgw = warp(img', itform, ImageTransformations.autorange(img, tform)) # nice, the image size more useful
imshow(imgw) # but the rectification is wrong!?