
using DelimitedFiles
using Flux

# load a specialized model format
function loadPyNNTxt(dest::AbstractString)
  mw = Vector{Array{Float32}}()
  @show files = readdir(dest)
  for f in files
    push!(mw, readdlm(joinpath(dest,f)))
  end
  return mw
end


##  Utility functions to take values from tf


function buildPyNNModel_01_FromElements(W1::AbstractMatrix{<:Real}=zeros(4,8),
                                        b1::AbstractVector{<:Real}=zeros(8),
                                        W2::AbstractMatrix{<:Real}=zeros(8,48),
                                        b2::AbstractVector{<:Real}=zeros(8),
                                        W3::AbstractMatrix{<:Real}=zeros(2,8),
                                        b3::AbstractVector{<:Real}=zeros(2))
  #
  # W1 = randn(Float32, 4,8)
  # b1 = randn(Float32,8)
  modjl = Chain(
    x -> (x*W1)' .+ b1 .|> relu,
    x -> reshape(x', 25,8,1),
    x -> maxpool(x, PoolDims(x, 4)),
    # x -> reshape(x[:,:,1]',1,:),
    x -> reshape(x[:,:,1]',:),
    Dense(48,8,relu),
    Dense(8,2)
  )

  modjl[5].W .= W2
  modjl[5].b .= b2

  modjl[6].W .= W3
  modjl[6].b .= b3

  return modjl
end

# As loaded from tensorflow get_weights
# Super specialized function
function buildPyNNModel_01_FromWeights(pywe)
  buildPyNNModel_01_FromElements(pywe[1], pywe[2][:], pywe[3]', pywe[4][:], pywe[5]', pywe[6][:])
end

# convenience function to load specific model format from tensorflow
function loadTfModelIntoFlux(dest::AbstractString)
  weights = loadPyNNTxt(dest::AbstractString)
  buildPyNNModel_01_FromWeights(weights)
end


## predictor functions


# @everywhere function predFluxOdo2(smpls::AbstractMatrix{<:Real},
#                                   allModelsLocal::Vector)
#   #
#
# end


# @everywhere function JlOdoPredictorPoint2(smpls::AbstractMatrix{<:Real},
#                                           allModelsLocal::Vector)
#   #
#   @warn "JlOdoPredictorPoint2 is now OBSOLETE"
#   arr = zeros(length(allModelsLocal), 2)
#   for i in 1:length(allModelsLocal)
#     arr[i,:] = allModelsLocal[i](smpls)
#   end
#   return arr
# end



#
