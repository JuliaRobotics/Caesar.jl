
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


#
