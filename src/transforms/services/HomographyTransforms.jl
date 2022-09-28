
## ============================================================
## FIXME, not-yet-consolidated rigid transform code that must be deprecated below
## ============================================================

# name euler here is very ambiguous, these are Lie algebra elements
# used to manipulate cartesian coordinates in a TranslationGroup(3) space.
function euler_angles_to_linearized_rotation_matrix(α1, α2, α3, rigid::Bool=true)
  dR = if rigid
    # TODO likely faster performance by using a retraction instead of expmap
    exp_lie(_SO3_MANI, hat(_SO3_MANI, SMatrix{3,3, Float64}(I), SA[α1, α2, α3]))
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
