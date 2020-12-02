# 

using LinearAlgebra
using Rotations, CoordinateTransformations

import Base: convert
import IncrementalInference: getSample


export Pose2AprilTag4Corners, PackedPose2AprilTag4Corners

# suppressing but listing some internal functions
# export _defaultCameraCalib, _AprilTagToPose2


"""
    $TYPEDEF

Simplified constructor type to convert between 4 corner detection of AprilTags to a Pose2Pose2 factor for use in 2D

Notes
- An assumption about camera or body frame is made, please open an issue at Caesar.jl for guidance on building more options.
- Helper constructor uses `fx,fy,cx,cy,s` to build `K`, 
  - setting `K` will overrule `fx,fy,cx,cy,s`.
- Finding preimage from deconv measurement sample `idx` in place of MvNormal mean:
  - `measPts = approxDeconv(dfg, fct)`
  - `obj = (x) -> fct.preimage[1](deconvMeasSol[idx], x)`
  - `result = Optim.optimize(obj, fct.preimage[2], BFGS())` and

DevNotes
- TODO IIF will get plumbing to combine many of preimage `obj` terms into single calibration search

Related

`AprilTags.detect`, `PackedPose2AprilTag4Corners`
"""
struct Pose2AprilTag4Corners{T <: SamplableBelief, F <: Function} <: AbstractRelativeRoots
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
  Zij::RoME.Pose2Pose2{T}

  # experimental development work for preimage parameter searching
  preimage::Tuple{F, Vector{Float64}}
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


"""
    $SIGNATURES

Notes
- assume body frame is xyz <==> fwd-lft-up
- assume AprilTags pose is xyz <==> rht-dwn-fwd
- FIXME, validate that: camera frame is xyz <==> row-col-bck <==> u-v-bck  <==> dwn-rht-bck
"""
function _AprilTagToPose2(corners, 
                          homography::AbstractMatrix{<:Real}, 
                          fx_::Real, 
                          fy_::Real, 
                          cx_::Real, 
                          cy_::Real, 
                          taglength_::Real)
  #
  pose, err1 = AprilTags.tagOrthogonalIteration(corners,homography, fx_, fy_, cx_, cy_, taglength = taglength_)
  cTt = LinearMap(pose[1:3, 1:3])∘Translation((pose[1:3,4])...)
  # camera to body rotation, 
  bRc = Rotations.Quat(1/sqrt(2),0,0,-1/sqrt(2))*Rotations.Quat(1/sqrt(2),-1/sqrt(2),0,0)
  # for tag in camera frame transform
  bTt = LinearMap(bRc) ∘ cTt
    # wTb = LinearMap(zT.R.R) ∘ Translation(zT.t...)
    # wTt = wTb ∘ bTt
  
  ld = LinearAlgebra.cross(bTt.linear*[0;0;1], [0;0;1])
  theta = atan(ld[2],ld[1])
  [bTt.translation[1:2,];theta]
end

const _CornerVecTuple = Union{NTuple{4,Tuple{Float64,Float64}}, Vector{Tuple{Float64,Float64}}, Vector{Vector{Float64}}}

function Pose2AprilTag4Corners(;corners::_CornerVecTuple=((0.0,0.0),(1.0,0.0),(0.0,1.0),(1.0,1.0)),
                                homography::AbstractMatrix{<:Real}=diagm(ones(3)),
                                fx::Real=524.040,
                                fy::Real=fx,
                                cx::Real=251.227,
                                cy::Real=319.254,
                                s::Real =0.0,
                                K::AbstractMatrix{<:Real}=_defaultCameraCalib(fx=fx,
                                                                              fy=fy,
                                                                              cy=cy,
                                                                              cx=cx,
                                                                              s=s),
                                id::Int=-1,
                                taglength::Real=0.25,
                                covariance::Matrix{<:Real}=diagm([0.1;0.1;0.1].^2) )
  #
  # make standard tuple
  corners_ = if corners isa Tuple
    corners
  else
    c = corners
    ((c[1][1],c[1][2]),(c[2][1],c[2][2]),(c[3][1],c[3][2]),(c[4][1],c[4][2]))
  end

  # calculate the transform
  # fx_, fy_, cx_, cy_ = K[1,1], K[2,2], K[1,3], K[2,3]
  x0 = [K[1,1], K[2,2], K[1,3], K[2,3], taglength]
  Dtag = _AprilTagToPose2(corners, homography, x0...)
  # println("$(tag.id), $(ld[3]), $(round.(Dtag, digits=3))")
  p2l2 = Pose2Pose2(MvNormal(Dtag, covariance))

  # preimage optimize function
  preImgFnc = (y, x) -> sum( (y - _AprilTagToPose2(corners_, homography, x...)).^2 )

  #
  return Pose2AprilTag4Corners(corners_, homography, K, taglength, id, p2l2, (preImgFnc, x0))
end





# just pass through sample and factor evaluations

getSample(pat4c::Pose2AprilTag4Corners, N::Int=1) = getSample(pat4c.Zij, N)

(pat4c::Pose2AprilTag4Corners)(x...) = pat4c.Zij(x...)


## serialization

struct PackedPose2AprilTag4Corners <: PackedInferenceType
  # format of serialized data
  mimeType::String
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

  return PackedPose2AprilTag4Corners( "/application/JuliaLang/PackedPose2AprilTag4Corners",
                                      corVec, 
                                      obj.homography[:],
                                      obj.K[:],
                                      obj.taglength,
                                      obj.id )
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
                                id=obj.id  )
end




#