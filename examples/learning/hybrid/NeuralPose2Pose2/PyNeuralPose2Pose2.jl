# PyNeuralPose2Pose2

using Random, Statistics
using DistributedFactorGraphs, TransformUtils
@everywhere using Random, Statistics
@everywhere using DistributedFactorGraphs, TransformUtils

@everywhere begin
import Base: convert
import IncrementalInference: getSample

struct PyNeuralPose2Pose2{P,D<:Vector,M<:SamplableBelief} <: FunctorPairwise
  predictFnc::P
  joyVelData::D
  naiveModel::M
  naiveFrac::Float64
  Zij::Pose2Pose2
  specialSampler::Function # special keyword field name used to invoke 'specialSampler' logic
end


function sampleNeuralPose2(nfb::PyNeuralPose2Pose2,
                          N::Int,
                          fmd::FactorMetadata,
                          Xi::DFGVariable,
                          Xj::DFGVariable)::Tuple
 #

  # calculate naive model and Predictive fraction of samples, respectively
  Nn = round(Int, nfb.naiveFrac*N)
  # calculate desired number of predicted values
  Np = N - Nn
  len = length(nfb.joyVelData) # expect this to be only 25 at developmenttime, likely to change

  # model samples (all for theta at this time)
  smpls_mAll = rand(nfb.naiveModel, N)

  # sample predictive fraction
  iT, jT = getTimestamp(Xi), getTimestamp(Xj)
  iPts, jPts = (getBelief(Xi) |> getPoints), (getBelief(Xj) |> getPoints)
  @assert size(jPts,2) == size(iPts,2) "sampleNeuralPose2 can currently only evaluate equal population   size variables"

  # calculate an average velocity component
  DT = jT - iT
  DXY = (@view jPts[1:2,:]) - (@view jPts[1:2,:])
  # rotate delta position from world to local iX frame
  for i in 1:size(iPts,2)
    DXY[1:2,i] .= TransformUtils.R(iPts[3,i])'*DXY[1:2,i]
  end
  # replace delta (velocity) values for this sampling
  mVXY = Statistics.mean(DXY, dims=2)
  # divide time to get velocity
  mVXY ./= 1e-3*DT.value
  mVXY[1] = isnan(mVXY[1]) ? 0.0 : mVXY[1]
  mVXY[2] = isnan(mVXY[2]) ? 0.0 : mVXY[2]

  for i in 1:len
    nfb.joyVelData[i][3:4] = mVXY
  end
  # and predict
      # A = [rand(4) for i in 1:25]
  smpls_pAll = nfb.predictFnc(nfb.joyVelData)

  # number of predictors to choose from, and choose random subset
  Npreds = size(smpls_pAll,1)
  allPreds = 1:Npreds |> collect
  # randomly select particles for prediction (with possible duplicates forwhen Np > size(iPts,2))
  Npp = Np < Npreds ? Np : Npreds
  Nnn = N - Npp
  selPreds = @view shuffle!(allPreds)[1:Npp] # TODO better in-place
  smpls_p = smpls_pAll[selPreds,:] # sample per row??
  smpls_p[:,3] .= smpls_mAll[3,Nnn+1:N] # use naive delta theta at this time

  # naive fraction
  smpls_n = @view smpls_mAll[:, 1:Nnn] # sample per column
  # join and shuffle predicted odo values
  shfSmpl = shuffle!(1:N |> collect)
  smpls = hcat(smpls_n, smpls_p')[:,shfSmpl]

  # @show Statistics.mean(smpls, dims=2)
  # @show maximum(smpls, dims=2)
  # @show minimum(smpls, dims=2)

  return (smpls, )
end

# Convenience function to help call the right constuctor
PyNeuralPose2Pose2(nn::P,
                   jvd::D,
                   md::M,
                   naiveFrac::Float64=0.4,
                   ss::Function=sampleNeuralPose2) where {P, M <: SamplableBelief, D <: Vector} = PyNeuralPose2Pose2{P,D,M}(nn,jvd,md,naiveFrac,Pose2Pose2(MvNormal(zeros(3),diagm(ones(3)))),ss )
#


function (nfb::PyNeuralPose2Pose2)(
            res::AbstractArray{<:Real},
            userdata::FactorMetadata,
            idx::Int,
            meas::Tuple{AbstractArray{<:Real},},
            Xi::AbstractArray{<:Real,2},
            Xj::AbstractArray{<:Real,2}  )
  #
  nfb.Zij(res,userdata,idx,meas,Xi,Xj)
  nothing
end



## packing converters

struct PackedPyNeuralPose2Pose2 <: AbstractPackedFactor
  joyVelData::Vector{Vector{Float64}}
  naiveModel::String
  naiveFrac::Float64
end


function convert(::Type{PyNeuralPose2Pose2}, d::PackedPyNeuralPose2Pose2)
  PyNeuralPose2Pose2(PyTFOdoPredictorPoint2,d.joyVelData,extractdistribution(d.naiveModel),d.naiveFrac)
end

function convert(::Type{PackedPyNeuralPose2Pose2}, d::PyNeuralPose2Pose2)
  PackedPyNeuralPose2Pose2(d.joyVelData, string(d.naiveModel), d.naiveFrac)
end

end # everywhere

#
