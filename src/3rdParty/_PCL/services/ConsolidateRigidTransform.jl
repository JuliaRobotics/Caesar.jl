
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

