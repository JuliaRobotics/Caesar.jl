
## ============================================================
## TODO, to be consolidated with upstream Manifolds.jl features
## ============================================================

# name euler here is very ambiguous, these are Lie algebra elements used to 
# manipulate cartesian coordinates in a TranslationGroup(3) (Euclidean) space.
function euler_angles_to_linearized_rotation_matrix(
  α1::Real, 
  α2::Real, 
  α3::Real, 
  rigid::Bool=true
)
  dR = if rigid
    # TODO likely faster performance by using a retraction instead of expmap
    exp_lie(_SO3_MANI, hat(_SO3_MANI, SMatrix{3,3, Float64}(I), SA[α1, α2, α3]))
  else
    SMatrix{3,3,Float64}(1.0,0,0,0,1,0,0,0,1) + 
      hat(_SO3_MANI, Identity(_SO3_MANI), SA[α1, α2, α3])
    # [ 1 -α3  α2
    #   α3   1 -α1
    #  -α2  α1   1]
    #   TBD, was this a small angle assumption?
  end
end

"""
    $SIGNATURES

Convert Euclidean (not sure why Euler) to homogeneous coordinates `XH` (projective space). 

Also see: [`homogeneous_coord_to_euler_coord`](@ref)
"""
euler_coord_to_homogeneous_coord(XE::AbstractMatrix{<:Number}) = [XE ones(size(XE, 1),1)]

"""
    $SIGNATURES

Convert homogeneous coordinates (projective space) to Euclidean `XE` (not sure why Euler). 

Also see: [`euler_coord_to_homogeneous_coord`](@ref)
"""
homogeneous_coord_to_euler_coord(XH::AbstractMatrix{<:Number}) = (XH[:,1:3]./XH[:,4])
