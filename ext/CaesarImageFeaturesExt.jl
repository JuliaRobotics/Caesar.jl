module CaesarImageFeaturesExt

## FIXME, consider consolidating around ImageIO extension instead.

@info "Caesar.jl is loading extension functionality using ImageFeatures.jl"

using ImageFeatures
using JSON3
using TensorCast

import Base: Dict
import LinearAlgebra: norm

import Caesar: AbstractDFG, BlobEntry, addData!, DFG.sha256

import Caesar: toDictFeatures

import Caesar: distancesKeypoints, sortMinimumsToDiagonal, addDataImgTracksFwdBck!, curateFeatureTracks


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





"""
    distancesKeypoints

Return matrices of distances between keypoints (rows A) and (columns B).
"""
function distancesKeypoints(
  A::AbstractVector,
  B::AbstractVector,
  _distance::Function = (a, b) -> norm((b[1]-a[1], b[2]-a[2]))
)

  M = zeros(length(A),length(B))
  for (i,a) in enumerate(A), (j,b) in enumerate(B)
    M[i,j] = _distance(a,b)
  end

  return M
end


"""
sortMinimumPerColumnThresholding

Sort a matrix by permuting columns such that minimum elements fall near diagonal.

Notes
- To get colidx below threshold: `colidx_keep = colidx[subthr]`
- To get `rowidx = minrow[colidx] .|> s->s[2]`

Example

```julia
using GLMakie

# matrix with values representing distances
Mdists = rand(80,70)
colidx, subthr, minrow = sortMinimumsToDiagonal(Mdists, 3.0)

# which idx to keep or ignore above threshold
colidx_keep, colidx_abov = colidx[subthr], colidx[(!).(subthr)]

# human readable plot of matrix
fig = GLMakie.Figure()
ax = GLMakie.Axis(fig[1,1], xlabel="B", ylabel="A")
plot!(ax, exp.(-(reverse(Mdists[:,[colidx_keep; colidx_abov]]',dims=2))).^2)
fig
```
"""
function sortMinimumsToDiagonal(
  M::AbstractMatrix{T},
  thr::Real = typemax(T)
) where {T <: Real}
  # helper function for min and arg per column of matrix
  minArgByCol(_M::AbstractMatrix{T}, rc=size(_M)) = (C->findmin(k->_M[k,C], 1:rc[1])).(1:rc[2])
  
  # min corresponding row
  minrow = minArgByCol(M)

  # sort columns by index of where min row element appears -- i.e. index to make diagonal matrix of minimums 
  rowidx = minrow .|> s->s[2]
  colidx = sortperm(rowidx)
  
  # select those values below threshold
  subthr = minrow .|> s -> s[1] < thr

  # return both below and above threshold sorted
  return colidx, subthr, minrow
end


function addDataImgTracksFwdBck!(
  dfg::AbstractDFG,
  vlbl::Symbol,
  blobstore::Symbol,
  bloblbl::Symbol,
  origin::AbstractString,
  img_tracks::Dict{Int,<:AbstractVector{<:AbstractVector{<:Real}}};
  description = "Image feature tracks forward backward $(length(img_tracks)) cv.KLT",
  mimeType = "/application/octet-stream/json; _type=JuliaLang.Dict{Int,Vector{Vector{Float32}}}"
)
  blob = Vector{UInt8}(JSON3.write(img_tracks))
  entry = BlobEntry(;
    label = bloblbl,
    blobstore,
    hash = bytes2hex(sha256(blob)),
    origin,
    size = length(blob),
    description,
    mimeType,
  )
  addData!(dfg, vlbl, entry, blob)
end




function curateFeatureTracks(
  tracks_A,
  tracks_B,
  img::AbstractMatrix,
  mimg::AbstractMatrix;
  pixThr::Real = 5.0,
  hammThr::Real = 0.35,
  fig = nothing
)

  @cast t_A[k,d] := tracks_A[k][d]
  @cast t_B[k,d] := tracks_B[k][d]

  # # fig = Figure()
  # height = size(img,1)
  # if !isnothing(fig)
  #   ax = GLMakie.Axis(fig[1,1])
  #   image!(ax, rotr90(mimg))
  #   scatter!(ax, t_A[:,1], height .- t_A[:,2], color="red", markersize=15)
  #   scatter!(ax, t_B[:,1], height .- t_B[:,2], color=RGBf(0.0, 0.0, 0.5))
  #   # scatter!(ax, t_qN2[:,1], height .- t_qN2[:,2], color=RGBf(0.0, 0.8, 0.5))
  #   # scatter!(ax, t_qN2[:,1], height .- t_qN2[:,2], color=RGBf(0.0, 0.8, 0.5))
  # end

  M_A_B = distancesKeypoints(tracks_A, tracks_B)

  colidx, subthr, minrow = sortMinimumsToDiagonal(M_A_B, pixThr)

  colidx_keep, colidx_abov = colidx[subthr], colidx[(!).(subthr)]
  rowidx_keep = minrow[colidx_keep] .|> s->s[2]

  if !isnothing(fig)
    # fig = GLMakie.Figure()
    ax = GLMakie.Axis(fig[2,1], xlabel="B=tracks_B", ylabel="A=tracks_A")
    plot!(ax, exp.(-(reverse(M_A_B[:,[colidx_keep; colidx_abov]]',dims=2))).^2)
  end
  # fig

  
  # plot track lines
  tracks_A_B = tracks_A[rowidx_keep] .=> tracks_B[colidx_keep]
  # if !isnothing(fig)
  #   # fig = GLMakie.Figure()
  #   ax = GLMakie.Axis(fig[3,1])
  #   image!(ax, rotr90(mimg))
  #   for tab in tracks_A_B
  #     lines!(ax, [tab[1][1];tab[2][1];], height .- [tab[1][2];tab[2][2];], linewidth=10)
  #   end
  # end

  # BRIEF descriptor pruning
  brief_params = BRIEF()

  keyps_A = map(s->CartesianIndex(round.(Int,s[[2;1]])...), (k->k[1]).(tracks_A_B))
  desc_A = []
  for pt in keyps_A
    _dsc, _kyp = ImageFeatures.create_descriptor(img, [pt], brief_params) # orientations, orb_params)
    push!(desc_A, _dsc)
  end
  mask_p = (s->0<length(s)).(desc_A)

  keyps_B = map(s->CartesianIndex(round.(Int,s[[2;1]])...), (k->k[2]).(tracks_A_B))
  desc_B = []
  for pt in keyps_B
    _dsc, _kyp = ImageFeatures.create_descriptor(img, [pt], brief_params) # orientations, orb_params)
    push!(desc_B, _dsc)
  end
  # desc_B, keyps_B_ = ImageFeatures.create_descriptor(img, keyps_B, brief_params) # orientations, orb_params)
  mask_qN1 = (s->0<length(s)).(desc_B)

  mask_pqN1 = (mask_p .& mask_qN1)
  hammdist = Float64[]
  for i in 1:length(desc_A)
    hdis = if mask_pqN1[i]
      ImageFeatures.hamming_distance(desc_A[i][1], desc_B[i][1])
    else
      999.0
    end
    push!(hammdist, hdis)
  end
  hammselect = hammdist .< hammThr

  # if !isnothing(fig)
  #   ax = GLMakie.Axis(fig[4,1], title="sum(hammselect)=$(sum(hammselect))")
  #   image!(ax, rotr90(mimg))
  #   for tpq in tracks_A_B[hammselect]
  #     lines!(ax, [tpq[1][1];tpq[2][1];], height .- [tpq[1][2];tpq[2][2];], linewidth=10)
  #   end
  # end

  idx_A_B = (1:length(tracks_A))[rowidx_keep] .=> (1:length(tracks_B))[colidx_keep]

  return tracks_A_B[hammselect], idx_A_B[hammselect]
end


end # module