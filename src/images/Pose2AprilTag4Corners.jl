# 

using LinearAlgebra
using Rotations, CoordinateTransformations

import Base: convert
import IncrementalInference: getSample


export Pose2AprilTag4Corners, PackedPose2AprilTag4Corners
export _defaultCameraCalib

"""
    $TYPEDEF

Simplified constructor type to convert between 4 corner detection of AprilTags to a Pose2Pose2 factor for use in 2D

Notes
- An assumption about camera or body frame is made, please open an issue at Caesar.jl for guidance on building more options.
- Helper constructor uses `fx,fy,cx,cy,s` to build `K`, 
  - setting `K` will overrule `fx,fy,cx,cy,s`.

Related

`AprilTags.detect`, `PackedPose2AprilTag4Corners`
"""
struct Pose2AprilTag4Corners{T <: SamplableBelief} <: AbstractRelativeRoots
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
  Zij::Pose2Pose2{T}
end

"""
    $SIGNATURES

A default camera calibration matrix, useful for reference and quick tests.

Notes
- Hartley, Zisserman, Multiple View Geometry, p.157

DevNotes
- TODO likely to move to a common camera models Pkg
- TODO consolidate with similar functions in RoME?
"""
function _defaultCameraCalib(;fx::Real = 524.040,
                              fy::Real = 524.040,
                              cy::Real = 319.254,
                              cx::Real = 251.227,
                              s::Real  = 0.0)
  #
  K = [fx s  cx;
        0 fy cy;
        0 0  1.0]
  #
  return K
end

function Pose2AprilTag4Corners(;corners::NTuple{4,Tuple{Float64,Float64}}=((0.0,0.0),(1.0,0.0),(0.0,1.0),(1.0,1.0)),
                                homography::AbstractMatrix{<:Real}=diagm(ones(3)),
                                fx::Real=524.040,
                                fy::Real=fx,
                                cy::Real=319.254,
                                cx::Real=251.227,
                                s::Real =0.0,
                                K::AbstractMatrix{<:Real}=_defaultCameraCalib(fx=fx,
                                                                              fy=fy,
                                                                              cy=cy,
                                                                              cx=cx,
                                                                              s=s),
                                id::Int=-1,
                                taglength::Real=0.25 )
  #
  # calculate the transform
  fx_, fy_, cx_, cy_ = K[1,1], K[2,2], K[1,3], K[2,3]
  pose, err1 = tagOrthogonalIteration(corners,homography, fx_, fy_, cx_, cy_, taglength = taglength)

  cTt = LinearMap(pose[1:3, 1:3])∘Translation((pose[1:3,4])...)
  bRc = Rotations.Quat(1/sqrt(2),0,0,-1/sqrt(2))*Rotations.Quat(1/sqrt(2),-1/sqrt(2),0,0)
  bTt = LinearMap(bRc) ∘ cTt
  # wTb = LinearMap(zT.R.R) ∘ Translation(zT.t...)
  # wTt = wTb ∘ bTt

  ld = LinearAlgebra.cross(bTt.linear*[0;0;1], [0;0;1])
  theta = atan(ld[2],ld[1])
  Dtag = [bTt.translation[1:2,];theta]
  # println("$(tag.id), $(ld[3]), $(round.(Dtag, digits=3))")
  p2l2 = Pose2Pose2(MvNormal(Dtag,diagm([0.1;0.1;0.1].^2)))

  #
  return Pose2AprilTag4Corners(corners, homography, K, taglength, id, p2l2)
end


function getSample(pat4c::Pose2AprilTag4Corners{T}, N::Int=1) where T

  return getSample(pat4c.Zij, N)
end


# just a pass through
(pat4c::Pose2AprilTag4Corners)(x...) = pat4c.Zij(x...)



## serialization

struct PackedPose2AprilTag4Corners <: PackedInferenceType
  corners::Vector{Float64}
  # homography matrix
  homography::Vector{Float64}
  # camera calibration
  K::Vector{Float64}
  # define the size of the tag in meters
  taglength::Float64
  # tag id
  id::Int
  # expected to be used in the future
  mimeType::String
end


function convert( ::Type{<:PackedInferenceType}, 
                  obj::Pose2AprilTag4Corners)
  #
  corVec = zeros(8)
  corVec[1] = obj.corners[1][1]
  corVec[2] = obj.corners[1][2]
  corVec[3] = obj.corners[2][1]
  corVec[4] = obj.corners[2][2]
  corVec[5] = obj.corners[3][1]
  corVec[6] = obj.corners[3][2]
  corVec[7] = obj.corners[4][1]
  corVec[8] = obj.corners[4][2]

  return PackedPose2AprilTag4Corners( corVec, 
                                      obj.homography[:],
                                      obj.K[:],
                                      obj.taglength,
                                      obj.id,
                                      "/application/PackedPose2AprilTag4Corners")
end


function convert( ::Type{<:DFG.AbstractRelative}, 
                  obj::PackedPose2AprilTag4Corners)
  #
  cv = obj.corners
  corners = ((cv[1],cv[2]),(cv[3],cv[4]),(cv[5],cv[6]),(cv[7],cv[8]))
  
  return Pose2AprilTag4Corners( corners=corners, 
                                homography=reshape(obj.homography,:,3), 
                                K=reshape(obj.K,3,3),
                                taglength=obj.taglength,
                                id=obj.id )
end




#