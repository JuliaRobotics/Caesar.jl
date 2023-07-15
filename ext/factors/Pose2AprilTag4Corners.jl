

"""
    $TYPEDEF

Simplified constructor type to convert between 4 corner detection of AprilTags to a Pose2Pose2 factor for use in 2D

Notes
- Coordinate frames are:
  - assume robotics body frame is xyz <==> fwd-lft-up
  - assume AprilTags pose is xyz <==> rht-dwn-fwd
  - assume camera frame is xyz <==> rht-dwn-fwd
  - assume Images.jl frame is row-col <==> i-j  <==> dwn-rht
- Helper constructor uses `f_width, f_height, c_width, c_height,s` to build `K`, 
  - setting `K` will overrule `f_width,f_height, c_width, c_height,s`.
- Finding preimage from deconv measurement sample `idx` in place of MvNormal mean:
  - see [`generateCostAprilTagsPreimageCalib`](@ref) for detauls.

Example

```julia
# bring in the packages
using AprilTags, Caesar, FileIO

# the size of the tag, as in the outer length of each side on of black square 
taglength = 0.15

# load the image
img = load("photo.jpg")

# the image size
width, height = size(img)
# auto-guess `f_width=height, c_width=round(Int,width/2), c_height=round(Int, height/2)`

detector = AprilTagDetector()
tags = detector(img)

# new factor graph with Pose2 `:x0` and a Prior.
fg = generateGraph_ZeroPose(varType=Pose2)

# use a construction helper to add factors to all the tags
for tag in tags
  tagSym = Symbol("tag\$(tag.id)")
  exists(fg, tagSym) ? nothing : addVariable!(fg, tagSym, Pose2)
  pat = Pose2AprilTag4Corners(corners=tag.p, homography=tag.H, taglength=taglength)
  addFactor!(fg, [:x0; tagSym], pat)
end

# free AprilTags library memory
freeDetector!(detector)
```

DevNotes
- TODO IIF will get plumbing to combine many of preimage `obj` terms into single calibration search

Related

`AprilTags.detect`, `PackedPose2AprilTag4Corners`, [`generateCostAprilTagsPreimageCalib`](@ref)
"""
struct Pose2AprilTag4Corners{T <: SamplableBelief, F <: Function} <: AbstractManifoldMinimize
  # 4 corners as detected by AprilTags
  corners::NTuple{4,Tuple{Float64,Float64}}
  # homography matrix
  homography::Matrix{Float64}
  # camera calibration
  K::Matrix{Float64}
  # define the size of the tag in meters
  taglength::Float64
  # tag id
  id::Int
  # internally computed relative factor between binary variables-- from camera to tag in camera or body frame
  Z::RoME.Pose2Pose2{T}

  # experimental for 'SLAM-aware' camera intrinsic calibration -- stores (lambda function, starting estimate), see [`generateCostAprilTagsPreimageCalib`](@ref).
  preimage::Tuple{F, Vector{Float64}}
end



Base.@kwdef struct PackedPose2AprilTag4Corners <: AbstractPackedFactor
  # format of serialized data
  _type::String = "Caesar.PackedPose2AprilTag4Corners"
  # corners, as detected by AprilTags library
  corners::Vector{Float64}
  # homography matrix
  homography::Vector{Float64}
  # camera calibration
  K::Vector{Float64}
  # define the size of the tag in meters
  taglength::Float64
  # tag id
  id::Int
end