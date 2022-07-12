

"""
    $TYPEDEF

Common functor for transforming a large number of points through 
`SpecialEuclidean(2)` or `SpecialEuclidean(3)` rigid body transform.

Notes
- Currently uses `Manifolds.affine_matrix`
- Used by `_PCL.apply` and `ScatterAlign`
"""
struct _FastTransform3D{M,T}
  rTo::SMatrix{4,4,T}
end

_FastTransform3D(M_::M, rPo::ArrayPartition, ::T=0.0) where {M <: typeof(SpecialEuclidean(3)),T<:Real} = _FastTransform3D{M,T}(SMatrix{4,4,T}(affine_matrix(M_,rPo)))

function _FastTransform3D(M_::M, rPo::ArrayPartition, ::T=0.0) where {M <: typeof(SpecialEuclidean(2)),T<:Real}
  rTo2 = affine_matrix(M_,rPo)
  rTo = zeros(T,4,4)
  rTo[4,4] = 1
  rTo[1:2,1:2] .= rTo2[1:2,1:2]
  rTo[1:2,4] .= rTo2[1:2,3]
  _FastTransform3D{M,T}(SMatrix{4,4,T}(rTo))
end

function (ft3::_FastTransform3D{<:typeof(SpecialEuclidean(3)),T})(
    src::AbstractVector{<:Real}
  ) where T
  #
  pV = SVector{4,T}(src...,1.0)
  wV = ft3.rTo*pV
  SVector{3,T}(wV[1],wV[2],wV[3])
end

function (ft3::_FastTransform3D{<:typeof(SpecialEuclidean(2)),T})(
    src::AbstractVector{<:Real}
  ) where T
  #
  pV = SVector{4,T}(src...,0.0,1.0)
  wV = ft3.rTo*pV
  SVector{2,T}(wV[1], wV[2])
end

