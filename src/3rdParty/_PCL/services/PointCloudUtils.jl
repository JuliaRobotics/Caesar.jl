
export getDataPointCloud, getPointCloud, getPointCloud2D, getPointCloud3D
export findObjectVariablesFromWorld, previewObjectSubcloudInLocalFromWorld
export calcAxes3D


function getDataPointCloud(
  nfg::AbstractDFG,
  varlbl, 
  pattern::Union{Symbol,UUID,<:AbstractString, Regex};
  getDatakws...
)
  # get point cloud blob
  try
    de,dbl = getData(nfg, Symbol(varlbl), pattern; getDatakws...)
    if isnothing(dbl)
      @error "could find in variable $varlbl, blob $pattern"
      return nothing
    end
    # FIXME, change serialization to more standard pcd or laz formats, see Caesar.jl #921
    return dbl |> IOBuffer |> Serialization.deserialize
  catch err
    if err isa KeyError
      @error err
      # Base.showerror(stdout, err, Base.catch_backtrace())
      return nothing
    end
    throw(err)
  end
  # unpack specific blob format
end



function getPointCloud(
  nfg::AbstractDFG,
  label,
  pattern = r"PCLPointCloud2", 
  w...;
  checkhash::Bool=true,
  kw...
)
  pc2_a = getDataPointCloud(nfg, label, pattern; checkhash)
  if pc2_a isa Nothing
    return nothing
  end
  pc_a = _PCL.PointCloud(pc2_a)
  return _prepPointCloud(pc_a, w...; kw...)
end

getPointCloud2D(
  nfg::AbstractDFG,
  label, 
  regex=r"PCLPointCloud2", 
  dims=1:2;
  minrange=0.0,
  maxrange=999.
) = getPointCloud(
  nfg, 
  label, 
  regex, 
  dims;
  minrange,
  maxrange
)

getPointCloud3D(
  nfg::AbstractDFG,
  label, 
  regex=r"PCLPointCloud2", 
  dims=1:3;
  minrange=0.0,
  maxrange=999.
) = getPointCloud(
  nfg, 
  label, 
  regex, 
  dims;
  minrange,
  maxrange
)

"""
    $SIGNATURES

Convenience wrapper around [`getSubcloud`](@ref), with a bounding box defined in 
local `p` frame coordinates (same as body frame if no lever-arm to sensor is required).
"""
function getDataSubcloudLocal(
  dfg::AbstractDFG,
  vlb::Symbol,
  p_BBo::_PCL.AbstractBoundingBox, # GeoB.Rect3,
  bllb::Union{Symbol, UUID, <:AbstractString, Regex} = r"PCLPointCloud2";
  checkhash::Bool=true,
  pointcloud = getDataPointCloud(dfg, vlb, bllb; checkhash) |> PointCloud
)
  #
  getSubcloud(pointcloud, p_BBo)
end

"""
    $SIGNATURES

Get an object subcloud from a specific pose point cloud, but specify the 
bounding box in world frame coordinates.  Useful for quickly pulling object
subclouds for nearly correct solutions.
"""
function getDataSubcloudLocalFromWorld(
  dfg::AbstractDFG,
  vlb::Symbol,
  w_BBo::_PCL.AbstractBoundingBox, # GeoB.Rect3,
  bllb::Union{Symbol, UUID, <:AbstractString, Regex} = r"PCLPointCloud2";
  solveKey::Symbol = :default,
  checkhash::Bool=true
)
  p_BBo, ohat_T_p = transformFromWorldToLocal(dfg, vlb, w_BBo; solveKey)
  getDataSubcloudLocal(dfg, vlb, p_BBo, bllb; checkhash), ohat_T_p
end

"""
    $SIGNATURES

Individually transform and return a merged list of point clouds into a common `::PointCloud` in the local frame from caller.
"""
function mergePointCloudsWithTransforms(
  l_Ts_bb::AbstractVector{<:ArrayPartition},
  bb_PCs::AbstractVector{<:_PCL.PointCloud},
)
  M = SpecialEuclidean(3)
  l_PC = _PCL.PointCloud()

  for (l_T_bb, bb_SC) in zip( l_Ts_bb, bb_PCs )
    l_SC = _PCL.apply(M, l_T_bb, bb_SC)
    cat(l_PC, l_SC; reuse=true)
  end

  # calculate new object frame
  l_T_o = _PCL.calcAxes3D(l_PC)
  o_T_l = inv(M, l_T_o)

  # calculate transforms from subcloud frames in new object frame o_T_bb
  o_Ts_li = map(s->compose(M, o_T_l, s), l_Ts_bb)

  # TODO, change to o_PC
  l_PC, o_Ts_li
end

"""
    $SIGNATURES

Point cloud leave-one-out (LOO) element alignment to a merged list of leave in elements (LIE) clouds.

Notes
- Takes in two lists, namely 
  - transforms (`ohat_T_l`) from some local frame `l` to a current best guess common object frame `ohat`,
  - point clouds in local frame `l`.
- Internal computations transform point cloud to object frame `o_PC = o_T_l ∘ l_PC`.
- The loo index argument is the element from lists which will be aligned to others.
"""
function alignPointCloudLOO!(
  ohat_Ts_l::AbstractVector{<:ArrayPartition},
  l_PCs::AbstractVector{<:PointCloud},
  loo_idx::Int;
  updateTloo::Bool=false,
  verbose=false,
  max_iterations = 25,
  correspondences = 500,
  neighbors = 50
)
  # For 3D transforms
  M = SpecialEuclidean(3)
  
  # prep the leave one out element
  ohat_PCloo = _PCL.apply( 
    M,
    ohat_Ts_l[loo_idx],
    l_PCs[loo_idx]
  )
  # prep the leave in elements (all but loo element)
  lie = setdiff(1:length(l_PCs), [loo_idx;])
  o_PClie, o_Ts_li = _PCL.mergePointCloudsWithTransforms(ohat_Ts_l[lie], l_PCs[lie])
  
  # do the alignment of loo onto lie
  o_Tloo_l_ = if length(ohat_Ts_l) == 1
    ohat_Ts_l[1]
  else
    o_Hloo_ohat, Hpts_mov, status = _PCL.alignICP_Simple(
      o_PClie,
      ohat_PCloo;
      verbose,
      max_iterations,
      correspondences,
      neighbors,
    )
    # convert SE affine H to tangent vector X for use in residual function 
    D_ = size(o_Hloo_ohat,2) - 1
    o_Tloo_ohat = ArrayPartition( 
      SVector(o_Hloo_ohat[1:D_,end]...), 
      SMatrix{D_,D_}(o_Hloo_ohat[1:D_,1:D_]) 
    )
    # that is, take the previous initial estimate and add the new alignment into it
    Manifolds.compose( M, o_Tloo_ohat, ohat_Ts_l[loo_idx] )
  end
  # make sure we have a static array (this line might be a repeat, but likely worth it)
  o_Tloo_l = ArrayPartition(
    SVector(o_Tloo_l_.x[1]...), 
    SMatrix{size(o_Tloo_l_.x[2])...}(o_Tloo_l_.x[2])
  )

  # to confirm the new alignment is good, transform loo cloud from scratch
  o_PCloo = _PCL.apply(
    M,
    o_Tloo_l,
    l_PCs[loo_idx]
  )
  
  if updateTloo
    @info "updateTloo $loo_idx"
    ohat_Ts_l[loo_idx] = o_Tloo_l
  end
  
  # TODO confirm expansion around e0, since PosePose factors expand around `q`
  return o_Tloo_l, o_PClie, o_PCloo
end

"""
    $SIGNATURES

Use sequential leave-one-out iteration strategy to self align 
assuming the list of point clouds have good coverage of the same object.

Notes
- Leave-one-out on the first iterLength number of elements.
  - Manually set `iterLength` to skip the tail elements of list -- e.g. skipping model priors.
"""
function alignPointCloudsLOOIters!(
  o_Ts_l::AbstractVector{<:ArrayPartition},
  l_PCs::AbstractVector{<:_PCL.PointCloud},
  updateTloo::Bool=true,
  MC::Int=3;
  iterLength::Int=length(l_PCs)
)
  o_Ts_l_ = (updateTloo ? s->s : deepcopy)(o_Ts_l)
  
  for m in 1:MC, k in 1:iterLength
    o_Tloo, o_PClie, o_PCloo = alignPointCloudLOO!(
      o_Ts_l_,
      l_PCs,
      k;
      updateTloo=true
    )
  end
  
  o_Ts_l_
end

"""
    $SIGNATURES

Find in the specified list of variables, which point clouds have at least `minpoint`s
within the bounding box.

Notes
- Uses Caesar.jl functionality
"""
function findObjectVariablesFromWorld(
  dfg::AbstractDFG,
  r_BBo::_PCL.AbstractBoundingBox;
  solveKey::Symbol = :default,
  minpoints::Int=5000,
  limit::Int=50,
  varPattern::Regex=r"x\d+",
  tags::AbstractVector{Symbol}=Symbol[],
  varList::AbstractVector{Symbol} = sort(ls(dfg,varPattern;tags); lt=DFG.natural_lt),
  blobLabel = r"PCLPointCloud2"
)
  M = SpecialEuclidean(3)
  objVars = Symbol[]
  
  for vlb in varList
    b_PC = _PCL.getDataPointCloud(dfg, vlb, blobLabel; checkhash=false) |> _PCL.PointCloud
    # TODO use PPE after PPEs are updated to Manifolds representation
    r_T_b = getBelief(dfg, vlb, solveKey) |> mean
    r_PC = _PCL.apply(M, r_T_b, b_PC)
    sc = _PCL.getSubcloud(r_PC, r_BBo)
    if minpoints < length(sc)
      push!(objVars, vlb)
    end
  end
  
  len = length(objVars)
  if 0<len
    idx = unique(Int.(round.(1:len/limit:len)))
    objVars[idx]
  else
    objVars
  end
end

#

function previewObjectSubcloudInLocalFromWorld(
  dfg::AbstractDFG,
  vlb::Symbol,
  w_BBo::AbstractBoundingBox;
  solveKey::Symbol = :default,
  blobLabel = r"PCLPointCloud2",
  checkhash::Bool=true
)
  p_PC = getDataPointCloud(dfg, vlb, blobLabel; checkhash) |> _PCL.PointCloud  
  p_BBo, o_T_p = transformFromWorldToLocal(dfg, vlb, w_BBo; solveKey)
  getSubcloud(p_PC, p_BBo)
end

"""
    $SIGNATURES

Calculate a new local to object (`l_T_o`) axis frame for a point cloud using PCA.

Example
```julia
M = SpecialEuclidean(3)
l_T_o = _PCL.calcAxes3D(l_PC)

# should return zeros when centered around the new object frame
iszeros = _PCL.calcAxes3D(_PCL.apply(M, inv(M, l_T_o), pc))
```
"""
function calcAxes3D(
  pc::PointCloud
)
  #
  xyz_ = (p->[p.x;p.y;p.z]).(pc.points)
  @cast xyz[d,i] := xyz_[i][d]
  mdl = fit(PCA, xyz; pratio=1)
  R = projection(mdl)
  μ = mean(xyz; dims=2)

  ArrayPartition(SVector(μ...), SMatrix{3,3}(R))
end

#