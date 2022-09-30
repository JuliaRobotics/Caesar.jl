
## ======================================================================================
## Deprecate below before Caesar.jl v0.15
## ======================================================================================

@deprecate create_homogeneous_transformation_matrix(R, t) affine_matrix(_SE3_MANI, ArrayPartition(t, R))


## ======================================================================================
## Deprecate below before Caesar.jl v0.14
## ======================================================================================

Base.@kwdef struct _FastRetract{M_ <: AbstractManifold, T}
  M::M_ = SpecialEuclidean(2)
  pTq::T = ProductRepr(MVector(0,0.0), MMatrix{2,2}(1.0, 0.0, 0.0, 1.0))
  p::ProductRepr{Tuple{SVector{2, Float64}, SMatrix{2, 2, Float64, 4}}} = ProductRepr(SA[0.0;0.0], SMatrix{2,2}(1.0, 0, 0, 1))
end

function (_user::_FastRetract)(pCq::AbstractVector{<:Real})
  retract!(_user.M, _user.pTq, _user.p, hat(_user.M, _user.p, pCq))

  return _user.pTq
end

