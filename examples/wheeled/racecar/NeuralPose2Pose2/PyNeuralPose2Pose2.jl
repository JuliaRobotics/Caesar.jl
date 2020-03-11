# PyNeuralPose2Pose2

import Base: convert
import IncrementalInference: getSample
using Random, Statistics

struct PyNeuralPose2Pose2{P,D<:Vector,M<:SamplableBelief} <: FunctorPairwiseMinimize
  Zij::Pose2Pose2
  predictFnc::P
  joyVelData::D
  naiveModel::M
  naiveFrac::Int
  specialSampler::Function
end


# special sampling function
function sampleNeuralPose2(nfb::PyNeuralPose2Pose2,
                           N::Int=1,
                           Xi::DFGVariable,
                           Xj::DFGVariable)::Tuple
  #
  @assert size(jPts,2) == size(iPts,2) "sampleNeuralPose2 can currently only evaluate equal population size variables"

  # calculate naive model and Predictive fraction of samples, respectively
  Nn = round(Int, nfb.naiveFrac*N)
  # calculate desired number of predicted values
  Np = N - Nn
  len = length(nfb.joyVelData) # expect this to be only 25 at development time, likely to change

  # model samples (all for theta at this time)
  smpls_mAll = rand(nfb.naiveModel, N)
  # naive fraction
  smpls_n = @view smpls_mAll[1:Nn, :] # sample per row??

  # sample predictive fraction
  iT, jT = getTimestamp(Xi), getTimestamp(Xj)
  iPts, jPts = (getKDE(Xi) |> getPoints), (getKDE(Xj) |> getPoints)

  # calculate an average velocity component
  DT = jT - iT
  DXY = (@view jPts[1:2,:]) - (@view jPts[1:2,:])
  VXY = DXY ./ DT
  # replace velocity values for this sampling
  mVXY = Statistics.mean(VXY, dims=2)
  for i in 1:len
    nfb.joyVelData[i][3:4] = mVXY
  end
  # and predict
      # A = [rand(4) for i in 1:25]
  smpls_pAll = nfb.predictFnc(nfb.joyVelData)

  # number of predictors to choose from, and choose random subset
  selPreds = @view shuffle!(1:len |> collect)[1:Np] # TODO better in-place
  # randomly select particles for prediction (with possible duplicates for when Np > size(iPts,2))
  smpls_p = smpls_pAll[selPreds,:] # sample per row??
  smpls_p[:,3] .= smpls_mAll[Nn+1:N,3] # use naive delta theta at this time

  # join and shuffle predicted odo values
  shfSmpl = shuffle!(1:N |> collect)
  smpls = hcat(smpls_n', smpls_p')[:,shfSmpl]

  return (smpls, )
end

# Convenience function to help call the right constuctor
PyNeuralPose2Pose2(z::T,
                   nn::P,
                   jvd::D,
                   md::M,
                   naiveFrac::Float64=0.4,
                   ss::Function=sampleNeuralPose2) where {M <: SamplableBelief, D <: Vector} = PyNeuralPose2Pose2{T,P,D,M}(z,nn,jvd,md,naiveFrac,ss)
#


function (nfb::PyNeuralPose2Pose2)(
            res::AbstractArray{<:Real},
            userdata::FactorMetadata,
            idx::Int,
            meas::Tuple{AbstractArray{<:Real},},
            Xi::AbstractArray{<:Real,2},
            Xj::AbstractArray{<:Real,2}  ) <: FunctorPairwise
  #
  nfb.Zij(res,userdata,idx,meas,Xi,Xj)
  nothing
end

#
