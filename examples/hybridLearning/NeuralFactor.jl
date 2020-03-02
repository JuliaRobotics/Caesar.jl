# neural network factor implementation

cd(@__DIR__)
using Pkg
pkg"activate ."

# load the necessary Python bindings
include(joinpath(@__DIR__,"PythonTensorFlowUsage.jl"))

# using Revise

using Flux
using DifferentialEquations, OrdinaryDiffEq
using DiffEqFlux
using Optim
using Zygote, DiffEqSensitivity



import Base: convert
import IncrementalInference: getSample

# export NeuralFactor


struct NeuralSetBinaryFactor{T<:Vector{Union{Chain,FastChain}},D} <: FunctorPairwiseMinimize
  Zij::T
  data::D
end

function getSample(nfb::NeuralSetBinaryFactor, N::Int=1)::Tuple
  # evaluate
  len = length(nfg.Zij)
  spmls = []
  lbls = Vector{Int}(undef, len)
  sel = Categorical(ones(len)*(1/len))
  for i in 1:N
    lb = rand(sel)
    lbls[i] = lb
    val = 0 # evaluate nfb.Zij[lb]
    push!(smpls, val)
  end
  return (smpls, lbls)
end

function (nfb::NeuralSetBinaryFactor)(
            res::Array{Float64},
            userdata::FactorMetadata,
            idx::Int,
            meas::Tuple,
            xi::Array{Float64,2},
            xj::Array{Float64,2}  )
  #
  len = size(xi, 1)
  fill!(res, 0.0)
  for i in 1:len
    res[1] += meas[1][i,idx] - (xj[i,idx] - xi[i,idx])
  end
  res[1]
end



#
