
@doc raw"""
    $SIGNATURES

Transform and put 2D pointcloud as [x,y] coordinates of a-frame into `aP_dest`, by transforming 
incoming b-frame points `bP_src` as [x,y] coords via the transform `aTb` describing the b-to-a (aka a-in-b)
relation.  Return the vector of points `aP_dest`.

````math
{}^a \begin{bmatrix} x, y \end{bmatrix} = {}^a_b \mathbf{T} \, {}^b \begin{bmatrix} x, y \end{bmatrix}
````

DevNotes
- Consolidate functionality with [`_FastTransform3D`](@ref)

"""
function _transformPointCloud!(
  # manifold
  M::Union{<:typeof(SpecialEuclidean(2)),<:typeof(SpecialEuclidean(3))},
  # destination points
  aP_dest::AbstractVector,
  # source points
  bP_src::AbstractVector,
  # transform coordinates
  aCb::AbstractVector{<:Real}; 
  # base point on manifold about which to do the transform
  e0::ArrayPartition = getPointIdentity(M),
  backward::Bool=false
)
  #
  
  aPb = retract(M, e0, hat(M, e0, aCb))
  aPb = backward ? inv(M,aPb) : aPb
  # can do 2d or 3d
  aTb = _FastTransform3D(M,aPb)
  aP_dest .= aTb.(bP_src)

  aP_dest
end

function _transformPointCloud(
  # manifold
  M::Union{<:typeof(SpecialEuclidean(2)),<:typeof(SpecialEuclidean(3))},
  # source points
  bP_src::AbstractVector,
  # transform coordinates
  aCb::AbstractVector{<:Real}; 
  kw...
)
  #
  #dest points
  aP_dest = Vector{MVector{length(bP_src[1]),Float64}}(undef,length(bP_src)) 
  _transformPointCloud!(M, aP_dest, bP_src, aCb; kw...)
end



## ============================================================
## Consolidated rigid transform standardized against Manifolds.jl
## ============================================================


# Works for transform of both 2D and 3D  point clouds
# FIXME, to optimize, this function will likely be slow
# TODO, consolidate with transformPointcloud(::ScatterAlign,..) function
function Manifolds.apply( 
  M_::Union{<:typeof(SpecialEuclidean(2)),<:typeof(SpecialEuclidean(3))},
  rPp::Manifolds.ArrayPartition,
  pc::PointCloud{T} 
) where T
  #

  # allocate destination
  _pc = PointCloud(;header=pc.header,
                    points = Vector{T}(),
                    width=pc.width,
                    height=pc.height,
                    is_dense=pc.is_dense,
                    sensor_origin_=pc.sensor_origin_,
                    sensor_orientation_=pc.sensor_orientation_ )
  #

  ft3 = _FastTransform3D(M_, rPp, 0f0)
  nc = M_ isa typeof(SpecialEuclidean(3)) ? 3 : 2
  _data = MVector(0f0,0f0,0f0,0f0)

  # rotate the elements from the old point cloud into new static memory locations
  # NOTE these types must match the types use for PointCloud and PointXYZ
  # TODO not the world's fastest implementation
  @inbounds for pt in pc.points
    _data[1:nc] = ft3(view(pt.data, 1:nc)) # TODO avoid references or allocation on heap 
    _data[4] = pt.data[4]
    npt = PointXYZ(;color=pt.color, data=SVector{4,eltype(pt.data)}(_data...))
    push!(_pc.points, npt )
  end

  # return the new point cloud
  return _pc
end

function Manifolds.apply(
  M::Union{<:typeof(SpecialEuclidean(2)),<:typeof(SpecialEuclidean(3))},
  a_T_r::ArrayPartition,
  r_BB::AbstractBoundingBox 
)
  
  _Hdim(::typeof(SpecialEuclidean(2))) = 3
  _Hdim(::typeof(SpecialEuclidean(3))) = 4
  _hdim = _Hdim(M)
  # get the transform from obj bounding box to some reference frame r
  _r_H_bb(::AxisAlignedBoundingBox) = SMatrix{_hdim,_hdim}(diagm(ones(_hdim)))
  _r_H_bb(_r_BB::OrientedBoundingBox) = inv(_r_BB.bb_H_r)
  
  a_H_bb = affine_matrix(M,a_T_r)*_r_H_bb(r_BB)
  # ohat_H_p = inv(p_H_bb)
  a_T_bb = ArrayPartition(a_H_bb[1:end-1,end], a_H_bb[1:end-1,1:end-1])
  
  OrientedBoundingBox(r_BB.origin, r_BB.widths, a_T_bb.x[1], a_T_bb.x[2])
end


# return the bounding box p_BBo
function transformFromWorldToLocal(
  dfg::AbstractDFG,
  vlbl::Symbol,
  w_BBo::_PCL.AbstractBoundingBox;
  solveKey=:default
)
  #  

  v = getVariable(dfg, vlbl)
  M = getManifold(v)

  # w_T_p58 = getBelief(sfg_, :x58, :parametric) |> mean
  # p58_T_w = inv(M, w_T_p58)
  # p58_BBo_02 = _PCL.apply( M, p58_T_w, w_BBo_02 )

  # TODO consider and consolidate with getPPESuggested or full belief
  w_T_p = getBelief(dfg, vlbl, solveKey) |> mean
    # b_Cwp = getPPESuggested(dfg, vlbl, solveKey)
    # w_T_p = exp(M, e0, hat(M, e0, b_Cwp))
    # p_T_w = inv(M, w_T_p)
    # p_H_w = SMatrix{4,4}(affine_matrix(M, p_T_w))
  _p_T_w = inv(M, w_T_p)
  p_T_w = ArrayPartition(SA[_p_T_w.x[1]...], SMatrix{size(_p_T_w.x[2])...}(_p_T_w.x[2]))

  (
    apply( M, p_T_w, w_BBo ),
    p_T_w
  )
end

  # _bb_H_r(::AxisAlignedBoundingBox) = SMatrix{4,4}(diagm(ones(4)))
  # _bb_H_r(::OrientedBoundingBox) = w_BBo.bb_H_r
  
  # p_P1 = p_H_w * SA[w_BBo.origin...; 1.]
  # p_P2 = p_H_w * SA[(w_BBo.origin+w_BBo.widths)...; 1.]
  
  # p_H_bb = p_H_w*inv(_bb_H_r(w_BBo))
  # ohat_H_p = inv(p_H_bb)
  # ohat_T_p = ArrayPartition(ohat_H_p[end,1:end-1], ohat_H_p[1:end-1,1:end-1])

  # pose to approximate object frame, ohat_T_p
  # NOTE, slightly weird transform in that world rotation and local translation are mixed, so one is inverted to get consistent left action
  # NOTE, assuming rectangular bounding box, make object frame the center of the volume
  # ohat_V_p = SA[(SA[w_T_p.x[1]...] - (w_BBo.origin+0.5*w_BBo.widths))...] # b_Cwp[1:3]...
  # ohat_T_p = ArrayPartition(ohat_V_p, SMatrix{3,3}(w_T_p.x[2]))
  
  # (
  #   OrientedBoundingBox( w_BBo.origin, w_BBo.widths, ohat_T_p.x[1], ohat_T_p.x[2] ) # p_P1[1:3], (p_P2-p_P1)[1:3] ), 
  #   ohat_T_p
  # )

## ============================================================
## FIXME, not-yet-consolidated rigid transform code that must be deprecated below
## ============================================================

function transform!(
  pc, 
  H::AbstractMatrix
)
  XInH = euler_coord_to_homogeneous_coord([pc.x pc.y pc.z])
  XOutH = transpose(H*XInH')
  XOut = homogeneous_coord_to_euler_coord(XOutH)

  for i in 1:size(XOut,1)
    data=SVector{4,eltype(pc.xyz.points[1].data)}(XOut[i,1],XOut[i,2],XOut[i,3],1)
    npt = PointXYZ(;data)
    pc.xyz.points[i] = npt
  end
  # _data[1:nc] = ft3(view(pt.data, 1:nc)) # TODO avoid references or allocation on heap 
  # _data[4] = pt.data[4]
  # npt = PointXYZ(;color=pt.color, data=SVector{4,eltype(pt.data)}(_data...))
  return pc
end

