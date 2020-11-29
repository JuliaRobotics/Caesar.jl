# 

import Base: convert
import IncrementalInference: getSample


export Pose2AprilTag4Corners, PackedPose2AprilTag4Corners


"""
    $TYPEDEF

Simplified constructor type to convert between 4 corner detection of AprilTags to a Pose2Pose2 factor for use in 2D

Notes
- An assumption about camera or body frame is made, please open an issue at Caesar.jl for guidance on building more options.
"""
struct Pose2AprilTag4Corners{T <: SamplableBelief} <: AbstractRelativeRoots
  # 4 corners as detected by AprilTags
  corners::NTuple{4,Tuple{Float64,Float64}}
  # internally computed relative factor between binary variables-- from camera to tag in camera or body frame
  Zij::Pose2Pose2{T}
end


function Pose2AprilTag4Corners(corners::NTuple{4,Tuple{Float64,Float64}}=((0.0,0.0),(1.0,0.0),(0.0,1.0),(1.0,1.0)))
  # calculate the transform

  # make a relative Pose2Pose factor
  p2p2 = Pose2Pose2(MvNormal([20;0;0.0], diagm(0.001*ones(3))))

  #
  return Pose2AprilTag4Corners(corners, p2p2)
end


function getSample(pat4c::Pose2AprilTag4Corners{T}, N::Int=1) where T

  return (rand(pat4c.Zij, N), )
end


# just a pass through
(pat4c::Pose2AprilTag4Corners)(x...) = pat4c.Zij(x...)



## serialization

struct PackedPose2AprilTag4Corners <: PackedInferenceType
  corners::Vector{Float64}
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

  return PackedPose2AprilTag4Corners(corVec, "/application/PackedPose2AprilTag4Corners")
end


function convert( ::Type{<:AbstractRelative}, 
                  obj::PackedPose2AprilTag4Corners)
  #
  cv = obj.corners
  corners = ((cv[1],cv[2]),(cv[3],cv[4]),(cv[5],cv[6]),(cv[7],cv[8]))
  
  return Pose2AprilTag4Corners(corners)
end




#