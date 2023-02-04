
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
    return dbl |> Serialization.deserialize
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
  maxrange=999.,
  kw...
) = getPointCloud(
  nfg, 
  label, 
  regex, 
  dims;
  minrange,
  maxrange,
  kw...
)

getPointCloud3D(
  nfg::AbstractDFG,
  label, 
  regex=r"PCLPointCloud2", 
  dims=1:3;
  minrange=0.0,
  maxrange=999.,
  kw...
) = getPointCloud(
  nfg, 
  label, 
  regex, 
  dims;
  minrange,
  maxrange,
  kw...
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

function getPointCloudInWorld(
  dfg::AbstractDFG,
  vlb::Symbol;
  solveKey::Symbol=:default,
  blobLabel::Regex=r"PCLPointCloud2",
  checkhash::Bool=false
)
  #
  M = SpecialEuclidean(3)
  e0 = ArrayPartition(SA[0;0;0.], SMatrix{3,3}(1,0,0,0,1,0,0,0,1.))
  p_PC = getDataPointCloud(dfg, vlb, blobLabel; checkhash) |> PointCloud
  w_Cp = getPPESuggested(dfg, vlb, solveKey)
  wTp = exp(M,e0,hat(M,e0,w_Cp))
  # wTp = ArrayPartition(SVector(wTp_.x[1]), SMatrix{3,3}(wTp_.x[2]))
  w_PC = apply(M, wTp, p_PC)
  
  w_PC
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
  bb_PCs::AbstractVector{<:_PCL.PointCloud};
  flipXY::Bool=true
)
  M = SpecialEuclidean(3)
  l_PC = _PCL.PointCloud()

  for (l_T_bb, bb_SC) in zip( l_Ts_bb, bb_PCs )
    l_SC = _PCL.apply(M, l_T_bb, bb_SC)
    cat(l_PC, l_SC; reuse=true)
  end

  # calculate new object frame
  l_T_o = _PCL.calcAxes3D(l_PC; flipXY)
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
  ohat_Ts_r::AbstractVector{<:ArrayPartition},
  r_PCs::AbstractVector{<:PointCloud},
  loo_idx::Int;
  updateTloo::Bool=false,
  verbose=false,
  max_iterations = 40,
  correspondences = 500,
  neighbors = 50
)
  # For 3D transforms
  M = SpecialEuclidean(3)
  
  # prep the leave one out element
  ohat_PCloo = _PCL.apply( 
    M,
    ohat_Ts_r[loo_idx],
    r_PCs[loo_idx]
  )
  # prep the leave in elements (all but loo element)
  lie = setdiff(1:length(r_PCs), [loo_idx;])
  o_PClie, o_Ts_li = _PCL.mergePointCloudsWithTransforms(ohat_Ts_r[lie], r_PCs[lie])
  
  # do the alignment of loo onto lie
  o_Tloo_r_ = if length(ohat_Ts_r) == 1
    # take the subcloud reference frame r as the local frame
    ohat_Ts_r[1]
  else
    o_Hloo_ohat, _o_PCloo_, status = _PCL.alignICP_Simple(
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
    Manifolds.compose( M, o_Tloo_ohat, ohat_Ts_r[loo_idx] )
  end
  # make sure we have a static array (this line might be a repeat, but likely worth it)
  o_Tloo_r = ArrayPartition(
    SVector(o_Tloo_r_.x[1]...), 
    SMatrix{size(o_Tloo_r_.x[2])...}(o_Tloo_r_.x[2])
  )

  # to confirm the new alignment is good, transform loo cloud from scratch
  # note should be the same as _o_PCloo_ above
  o_PCloo = _PCL.apply(
    M,
    o_Tloo_r,
    r_PCs[loo_idx]
  )
  
  if updateTloo
    @info "updateTloo $loo_idx"
    ohat_Ts_r[loo_idx] = o_Tloo_r
  end
  
  # TODO confirm expansion around e0, since PosePose factors expand around `q`
  return o_Tloo_r, o_PClie, o_PCloo
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
  iterLength::Int=length(l_PCs),
  lookws...
)
  o_Ts_l_ = (updateTloo ? s->s : deepcopy)(o_Ts_l)
  
  for m in 1:MC, k in 1:iterLength
    o_Tloo, o_PClie, o_PCloo = alignPointCloudLOO!(
      o_Ts_l_,
      l_PCs,
      k;
      updateTloo=true,
      lookws...
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
  limit::Int=10,
  selection::Symbol = :biggest,
  varPattern::Regex=r"x\d+",
  tags::AbstractVector{Symbol}=Symbol[],
  minList::Int = 1,
  maxList::Int = 999999,
  minrange=1.0,
  varList::AbstractVector{Symbol} = sort(ls(dfg,varPattern;tags); lt=DFG.natural_lt)[minList:minimum([end;maxList])],
  blobLabel = r"PCLPointCloud2",
  sortList::Bool=true,
  checkhash::Bool=false
)
  M = SpecialEuclidean(3)
  len = length(varList)
  objVars_ = Vector{Symbol}(undef, len)
  numPts_  = Vector{Int}(undef, len)
  tsk = Vector{Task}(undef, len)

  for (i,vlb) in enumerate(varList)
    tsk[i] = Threads.@spawn begin
      # b_PC = _PCL.getDataPointCloud(dfg, $vlb, blobLabel; checkhash=false) |> _PCL.PointCloud
      b_PC = _PCL.getPointCloud3D(dfg, $vlb, blobLabel; checkhash, minrange) |> _PCL.PointCloud
      # TODO use PPE after PPEs are updated to Manifolds representation
      r_T_b = getBelief(dfg, $vlb, solveKey) |> mean
      r_PC = _PCL.apply(M, r_T_b, b_PC)
      sc = _PCL.getSubcloud(r_PC, r_BBo)
      numpts = length(sc)
      if minpoints < numpts
        objVars_[$i] = $vlb
        numPts_[$i] = numpts
      end
    end
  end
  wait.(tsk)
  
  objVars = Vector{Symbol}()
  numPts  = Vector{Int}()

  for i in 1:len
    if isassigned(objVars_, i)
      push!(objVars, objVars_[i])
      push!(numPts, numPts_[i])
    end
  end
  len = length(objVars)
  
  ovlbs = if 0<len
    if selection == :spread
      # evenly select poses to object (assuming enough coverage)
      idx = unique(Int.(round.(1:len/limit:len)))
      objVars[idx]
    elseif selection == :biggest
      prm = sortperm(numPts; rev = true)
      idx = 1:minimum([len; limit])
      objVars[prm[idx]]
    end
  else
    objVars
  end
  # usually sorted list is beter
  if sortList
    sort(ovlbs; lt=DFG.natural_lt)
  else
    ovlbs
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
  pc::PointCloud;
  flipXY::Bool=false
)
  #
  if 0 === length(pc)
    @warn "Cannot calculate a default axis for empty point cloud"
    return ArrayPartition(SVector(0,0,0.), SMatrix{3,3}(1,0,0,0,1,0,0,0,1.))
  end
  xyz_ = (p->[p.x;p.y;p.z]).(pc.points)
  @cast xyz[d,i] := xyz_[i][d]
  mdl = fit(PCA, xyz; pratio=1)
  # negative because during testing found rotation to have determinant -1
  R = projection(mdl)
  R .*= det(R) < 0 ? -1 : 1
  @assert isapprox(1, det(R); atol=0.1) "PCA derived rotation matrix should have determinant 1, not $(det(R))."
  μ = mean(xyz; dims=2)

  R_enh = if !flipXY
    # keep the direct PCA R as is
    R
  else
    # try pick a z (vertical) orientation closer to incoming transform
    Mr = SpecialOrthogonal(3)
    R0 = SMatrix{3,3,Float64}(pc.sensor_orientation_)
    Rx = _Rot.RotX(pi)*R
    Ry = _Rot.RotY(pi)*R
    costs = Float64[]
    arr = Matrix{Float64}[]
    for r in [R, Rx, Ry]
      try
        push!(costs, distance(Mr, R0, r))
        push!(arr, r)
      catch e
        # ERROR: ArgumentError: invalid index: nothing of type Nothing
        # [6] log!(M::Rotations{3}, X::MMatrix{3, 3, Float64, 9}, p::SMatrix{3, 3, Float64, 9}, q::Matrix{Float64})
        # @ Manifolds ~/.julia/packages/Manifolds/7JKQJ/src/manifolds/GeneralUnitaryMatrices.jl:560
        e isa ArgumentError ? nothing : rethrow(e)
      end
    end
    # costs = [distance(Mr, R0, R); distance(Mr, R0, Rx); distance(Mr, R0, Ry)]
    if 0 < length(costs)
      arr[argmin(costs)]
    else
      @warn "not able to approximate an orientation for the new object, using default idenity"
      R0
    end
  end

  ArrayPartition(SVector(μ...), SMatrix{3,3}(R_enh))
end

function Base.convert(
  ::Type{<:PointXYZ{C,F1}},
  pt::PointXYZ{C,F2}
) where {C, F1 <: Real, F2 <: Real}
  #
  PointXYZ(;
    color=pt.color, 
    data=SVector{4,F1}(pt.data...)
  )
end


function Base.convert(
  ::Type{<:PointCloud{<:PointXYZ{C,F1},S,M}},
  pc::PointCloud{<:PointXYZ{C,F2}}
) where {C, F1 <: Real, F2 <: Real, S, M}
  #
  if 0 ==length(pc)
    return PointCloud{PointXYZ{C, F1}, S, M}()
  end
  xyz_ = (p->F1[F1(p.x);F1(p.y);F1(p.z)]).(pc.points)
  @cast xyz[i,d] := xyz_[i][d]
  PointCloud(xyz)
end

"""
    $SIGNATURES

Return a PointCloud with all the points in the world frame coordinates, given the solveKey.

See also: [`saveLAS`](@ref)
"""
function exportPointCloudWorld(
  dfg::AbstractDFG;
  varList::AbstractVector{Symbol} = sort(ls(dfg); lt=DFG.natural_lt),
  solveKey::Symbol = :default,
  # TODO update to blobs saved as LAS files instead
  getpointcloud::Function = (v)->_PCL.getDataPointCloud(dfg, v, Regex("PCLPointCloud2"); checkhash=false),
  downsample::Int=1,
  minrange = 0.0,
  maxrange = 9999.0,
)
  M = SpecialEuclidean(3)
  ϵ0 = ArrayPartition(SVector(0,0,0.),SMatrix{3,3}(1,0,0,0,1,0,0,0,1.)) # MJL.identity_element(M)
  pc_map = _PCL.PointCloud()
  # loop through all variables in the given list
  for vl in varList
    pc_ = getpointcloud(vl)
    if pc_ isa Nothing
      @warn "Skipping variable without point cloud" vl
      continue
    end
    pc = PointCloud(pc_)
    
    pts_a = (s->[s.x;s.y;s.z]).(pc.points)
    pts_a = _filterMinRange(pts_a, minrange, maxrange)
    pc = PointCloud(pts_a)

    v = getVariable(dfg, Symbol(vl))
    if !(:parametric in listSolveKeys(v))
      @warn "Skipping $vl which does not have solveKey :parametric"
      continue
    end
    w_Cwp = calcPPE(v; solveKey).suggested
    wPp = Manifolds.exp(M,ϵ0,Manifolds.hat(M,ϵ0,w_Cwp))
    # wPp = getSolverData(v, solveKey).val[1]
    wPC = apply(M, wPp, pc)
    cat(pc_map, wPC; reuse=true)
  end

  pc_map
end

#