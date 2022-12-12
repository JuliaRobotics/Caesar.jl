
export getDataPointCloud, getPointCloud, getPointCloud2D, getPointCloud3D


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
  kw...
)
  pc2_a = getDataPointCloud(nfg, label, pattern)
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
  p_BBo::GeoB.Rect3,
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
  w_BBo::GeoB.Rect3,
  bllb::Union{Symbol, UUID, <:AbstractString, Regex} = r"PCLPointCloud2";
  solveKey::Symbol = :default,
  checkhash::Bool=true
)
  p_BBo = transformFromWorldToLocal(dfg, vlb, w_BBo; solveKey)
  getDataSubcloudLocal(dfg, vlb, p_BBo, bllb; checkhash)
end

"""
    $SIGNATURES

Individually transform and return a merged list of point clouds into one common `::PointCloud` object.
"""
function mergePointCloudsWithTransforms(
  o_Ts_l::AbstractVector{<:ArrayPartition},
  o_PCs::AbstractVector{<:_PCL.PointCloud},
)
  M = SpecialEuclidean(3)
  o_PC = _PCL.PointCloud()

  for (i, l_SC) in enumerate( o_PCs )
    o_SC = _PCL.apply(M, o_Ts_l[i], l_SC)
    cat(o_PC, o_SC; reuse=true)
  end

  o_PC
end

"""
    $SIGNATURES

Point cloud alignment of a leave-one-out (LOO) element to a merged list of leave in elements (LIE).
"""
function alignPointCloudLOO!(
  o_Ts_hato::AbstractVector{<:ArrayPartition},
  hato_PCs::AbstractVector,
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
  oo_PCloo = _PCL.apply( 
    M,
    o_Ts_hato[loo_idx],
    hato_PCs[loo_idx]
  )
  # prep the leave in elements (all but loo element)
  lie = setdiff(1:length(hato_PCs), [loo_idx;])
  o_PClie = _PCL.mergePointCloudsWithTransforms(o_Ts_hato[lie], hato_PCs[lie])
  
  # do the alignment of loo onto lie
  o_Hloo_oo, Hpts_mov, status = _PCL.alignICP_Simple(
    o_PClie, 
    oo_PCloo; 
    verbose,
    max_iterations,
    correspondences,
    neighbors,
  )
  # convert SE affine H to tangent vector X for use in residual function 
  o_Tloo_oo = ArrayPartition(o_Hloo_oo[1:end-1,end],o_Hloo_oo[1:end-1,1:end-1])
  # that is, take the previous initial estimate and add the new alignment into it
  o_Tloo_hato = Manifolds.compose(M, o_Tloo_oo, o_Ts_hato[loo_idx])
  
  # to confirm the new alignment is good, transform loo cloud from scratsh
  o_PCloo = _PCL.apply( 
    M,
    o_Tloo_hato,
    hato_PCs[loo_idx]
  )
  
  if updateTloo
    @info "updateTloo"
    o_Ts_hato[loo_idx] = o_Tloo_hato
  end
  
  # TODO confirm expansion around e0, since PosePose factors expand around `q`
  return o_Tloo_hato, o_PClie, o_PCloo
end

"""
    $SIGNATURES

Use sequential leave-one-out iteration strategy to self align 
assuming the list of point clouds have good coverage of the same object.
"""
function alignPointCloudsLOOIters!(
  o_Ts_l::AbstractVector{<:ArrayPartition},
  l_PCs::AbstractVector{<:_PCL.PointCloud},
  updateTloo::Bool=true,
  MC::Int=3,
)
  o_Ts_l_ = (updateTloo ? s->s : deepcopy)(o_Ts_l)

  for m in 1:MC, k in 1:length(l_PCs)
    o_Tloo, o_PClie, o_PCloo = alignPointCloudLOO!(
      o_Ts_l_,
      l_PCs,
      k;
      updateTloo=true
    )
  end
  
  o_Ts_l_
end

#