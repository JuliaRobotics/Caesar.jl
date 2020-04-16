
using DelimitedFiles

function loadPyNNTxt(dest::AbstractString)
  mw = Vector{Array{Float32}}()
  @show files = readdir(dest)
  for f in files
    push!(mw, readdlm(joinpath(dest,f)))
  end
  return mw
end
