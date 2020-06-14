
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



# convenience function to load specific model format from tensorflow
function loadPose2OdoNNModelIntoFlux(dest::AbstractString)
  weights = loadPyNNTxt(dest::AbstractString)
  buildPose2OdoNN_01_FromWeights(weights)
end


# function buildPose2OdoNN_01_FromWeightsGPU(pywe)
#   buildPose2OdoNN_01_FromElements(collect(pywe[1]) |> gpu, collect(pywe[2][:]) |> gpu, collect(pywe[3]') |> gpu, collect(pywe[4][:]) |> gpu, collect(pywe[5]') |> gpu, collect(pywe[6][:]) |> gpu) |> gpu
# end
#
# function loadPose2OdoNNModelIntoFluxGPU(dest::AbstractString)
#   weights = loadPyNNTxt(dest::AbstractString)
#   buildPose2OdoNN_01_FromWeightsGPU(weights)
# end

#
