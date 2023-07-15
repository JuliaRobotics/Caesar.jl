module CaesarImageFeaturesExt

@info "Caesar.jl is loading extension functionality using ImageFeatures.jl"

using ImageFeatures
using JSON3

import Base: Dict

import Caesar: toDictFeatures


function Base.Dict(
  orb::ImageFeatures.ORB
)
  jstr = JSON3.write(orb)
  dict = JSON3.read(jstr, Dict{String,Any})
  dict["_type"] = "ImageFeatures.ORB;params"
  dict
end

function toDictFeatures(
  orb::ImageFeatures.ORB, 
  descriptors::Union{<:AbstractVector{<:BitVector},<:AbstractVector{<:Integer}},
  keypoints::Union{<:AbstractVector{<:CartesianIndex},<:AbstractVector{<:Tuple},<:AbstractVector{<:AbstractVector}};
  metadata::Dict = Dict{String,Any}()
)
  #
  _totuple(s::Tuple) = s
  _totuple(s::AbstractVector) = tuple(s...)
  _totuple(s::CartesianIndex) = tuple(s[1],s[2])

  desci = descriptors .|> s->Int.(s)
  keyt = _totuple.(keypoints)
  
  dict = Dict{String,Any}()
  dict["ORBparams"] = Dict(orb)
  dict["descriptors"] = desci
  dict["keypoints"] = keyt

  if 0 < length(metadata)
    # add metadata
    dict["metadata"] = metadata
  end

  dict
end

# function fromFeaturesDict(
#   dict
# )
#   #


end # module