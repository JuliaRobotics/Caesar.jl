# Utils based on GeometryBasics

# 
function transformFromWorldToLocal(
  dfg::AbstractDFG,
  vlbl::Symbol,
  w_BBo::_PCL.AbstractBoundingBox; #GeoB.HyperRectangle;
  solveKey=:default
)
  #  
  v = getVariable(dfg, vlbl)
  # vt = getVariableType(v)
  M = getManifold(v)
  e0 = ArrayPartition(SA[0;0;0.], SMatrix{3,3}(diagm([1;1;1.])))
  
  b_Cwp = getPPESuggested(dfg, vlbl, solveKey)
  w_T_p = exp(M, e0, hat(M, e0, b_Cwp))
  p_T_w = inv(M, w_T_p)
  p_H_w = SMatrix{4,4}(affine_matrix(M, p_T_w))
  
  p_P1 = p_H_w * SA[w_BBo.origin...; 1.]
  p_P2 = p_H_w * SA[(w_BBo.origin+w_BBo.widths)...; 1.]
  
  # pose to approximate object frame, ohat_T_p
  # NOTE, slightly weird transform in that world rotation and local translation are mixed, so one is inverted to get consistent left action
  # NOTE, assuming rectangular bounding box, make object frame the center of the volume
  ohat_V_p = SA[(SA[b_Cwp[1:3]...] - (w_BBo.origin+0.5*w_BBo.widths))...]
  ohat_T_p = ArrayPartition(ohat_V_p, SMatrix{3,3}(w_T_p.x[2]))
  
  (
    AxisAlignedBoundingBox( GeoB.Rect3( GeoB.Point3(p_P1[1:3]...), GeoB.Point3((p_P2-p_P1)[1:3]...)) ), 
    ohat_T_p
  )
end

"""
    $SIGNATURES

Get the eight corners of bounding box in it's local coordinates.

See also: [`plotBoundingBox`](@ref)
"""
function getCorners(
  BB::_PCL.AbstractBoundingBox
)
  # https://math.stackexchange.com/questions/1472049/check-if-a-point-is-inside-a-rectangular-shaped-area-3d
  hr = BB.hr
  x_, y_, z_ = hr.origin[1], hr.origin[2], hr.origin[3]
  dx, dy, dz = hr.widths[1], hr.widths[2], hr.widths[3]

  p1 = [x_;y_;z_]
  p2 = [x_;y_+dy;z_]
  p4 = [x_+dx;y_;z_]
  p3 = [x_+dx;y_+dy;z_]

  p5 = [x_;y_;z_+dz]
  p6 = [x_;y_+dy;z_+dz]
  p8 = [x_+dx;y_;z_+dz]
  p7 = [x_+dx;y_+dy;z_+dz]

  corners_ = [p1,p2,p3,p4,p5,p6,p7,p8]

  _r_H_bb(::_PCL.AxisAlignedBoundingBox) = SMatrix{4,4}(diagm(ones(4)))
  _r_H_bb(obb::_PCL.OrientedBoundingBox) = inv(obb.bb_H_r)
  r_H_bb = _r_H_bb(BB)
  (s->(r_H_bb*[s...;1.])[1:3]).(corners_)
end

"""
    $SIGNATURES

Is point p inside the HyperRectangle.
"""
function inside(
  hr::GeoB.HyperRectangle, 
  p #::GeoB.Point
)
  #
  _p0,_p1 = GeoB.minmax(GeoB.Point(p...), hr.origin, hr.origin + hr.widths)
  _p0 == hr.origin && hr.widths == (_p1-_p0)
end

# inside(
#   aabb::AxisAlignedBoundingBox, 
#   p
# ) = inside(aabb.hr, p)


function inside(
  obb::AbstractBoundingBox, 
  pt
)
# https://math.stackexchange.com/questions/1472049/check-if-a-point-is-inside-a-rectangular-shaped-area-3d
  p = getCorners(obb)
  
  _i = p[2] - p[1]
  _j = p[4] - p[1]
  _k = p[5] - p[1]
  _v = pt   - p[1]

  # check if inside cuboid
  (0 < _v'*_i < _i'*_i) && (0 < _v'*_j < _j'_j) && (0 < _v'*_k < _k'*_k)
end


"""
    $SIGNATURES

Create a new subcloud containing the portion of the full point cloud that is inside the mask.
"""
function getSubcloud(
  pc_full::PointCloud,       # the full point cloud
  mask::AbstractBoundingBox  # starting out with GeometryBasics.Rect3
)
  objmask = map(s->_PCL.inside(mask, s.data[1:3]), pc_full.points)
  spt = (s->s.data[1:3]).(pc_full.points[objmask])
  @cast pts[i,d] := spt[i][d]
  _PCL.PointCloud(pts)
end



#