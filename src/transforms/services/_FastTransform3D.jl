

"""
    $TYPEDEF

Common functor for transforming a large number of points through 
`SpecialEuclideanGroup(2)` or `SpecialEuclideanGroup(3)` rigid body transform.

Notes
- Currently uses `Manifolds.affine_matrix`
- Used by `_PCL.apply` and `ScatterAlign`
"""
struct _FastTransform3D{M,N,T}
  rTo::SMatrix{N,N,T}
end

# _FastTransform3D(M_::M, rPo::ArrayPartition, ::T=0.0) where {M <: LieGroup,T<:Real} = _FastTransform3D{M,T}(SMatrix{4,4,T}(affine_matrix(M_,rPo)))

function _FastTransform3D(M_::M, rPo::ArrayPartition, ::T=0.0) where {M <: LieGroup,T<:Real}

  d = length(rPo.x[1])
  function _affine_matrix(::LieGroup,p::ArrayPartition)
    t,R = p.x[1], p.x[2]
    R_ = vcat(R, zeros(T, 1, d))
    v = [reshape(R_,1,:)..., t..., 1.0]
    SMatrix{d+1,d+1,T}(v)
  end
  # _affine_matrix(::LieGroup, m) = m
  rTo2 = _affine_matrix(M_,rPo)
  # rTo = zeros(T,4,4)
  # rTo[4,4] = 1
  # rTo[1:2,1:2] .= rTo2[1:2,1:2]
  # rTo[1:2,4] .= rTo2[1:2,3]
  _FastTransform3D{M,d+1,T}(rTo2)
end

# function (ft3::_FastTransform3D{<:LieGroup,T})(
#     src::AbstractVector{<:Real}
#   ) where T
#   #
#   pV = SVector{4,T}(src...,1.0)
#   wV = ft3.rTo*pV
#   SVector{3,T}(wV[1],wV[2],wV[3])
# end

function (ft3::_FastTransform3D{<:LieGroup,N,T})(
    src::AbstractVector{<:Real}
  ) where {N,T}
  #
  d = N-1
  pV = SVector{N,T}(src...,1.0)
  wV = ft3.rTo*pV
  SVector{d,T}(wV[1:d]...)
end

