

## ============================================================
## Consolidated rigid transform standardized against Manifolds.jl
## ============================================================


# Works for transform of both 2D and 3D  point clouds
# FIXME, to optimize, this function will likely be slow
# TODO, consolidate with transformPointcloud(::ScatterAlign,..) function
function apply( M_::Union{<:typeof(SpecialEuclidean(2)),<:typeof(SpecialEuclidean(3))},
                rPp::Manifolds.ArrayPartition,
                pc::PointCloud{T} ) where T
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

const _SO3_MANI = SpecialOrthogonal(3)
const _SE3_MANI = SpecialEuclidean(3)

# name euler here is very ambiguous, these are Lie algebra elements
# used to manipulate cartesian coordinates in a TranslationGroup(3) space.
function euler_angles_to_linearized_rotation_matrix(α1, α2, α3, rigid::Bool=true)
  dR = if rigid
    # TODO likely faster performance by using a retraction instead of expmap
    exp_lie(_SO3_MANI, hat(_SO3_MANI, Identity(_SO3_MANI), SA[α1, α2, α3]))
  else
    SMatrix{3,3,Float64}(1.0,0,0,0,1,0,0,0,1) + 
      hat(_SO3_MANI, Identity(_SO3_MANI), SA[α1, α2, α3])
    # [ 1 -α3  α2
    #   α3   1 -α1
    #  -α2  α1   1]
  end
end

function create_homogeneous_transformation_matrix(R, t)
  H = affine_matrix(_SE3_MANI, ArrayPartition(t, R))
  # H = [R          t
  #      zeros(1,3) 1]
end
function euler_coord_to_homogeneous_coord(XE)
  no_points = size(XE, 1)
  XH = [XE ones(no_points,1)]
end
function homogeneous_coord_to_euler_coord(XH)
  XE = XH[:,1:3]./XH[:,4]
end

function transform!(pc, H)
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

