# 

import IncrementalInference: getSample


struct Pose2AprilTag4Corners{T} <: AbstractRelativeRoots
    corners::NTuple{4,Tuple{Float64,Float64}}
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


function (pat4c::Pose2AprilTag4Corners)(res::AbstractVector{<:Real},
                                        fmd::FactorMetadata,
                                        idx::Int,
                                        meas::Tuple,
                                        X1::AbstractArray,
                                        X2::AbstractArray )
  #
  pat4c.Zij( res, fmd, idx, meas, X1, X2 )
end











#