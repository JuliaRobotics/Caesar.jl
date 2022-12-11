# Utils based on GeometryBasics

# 
function transformFromWorldToLocal(
  dfg::AbstractDFG,
  vlbl::Symbol,
  w_BBo::GeoB.HyperRectangle;
  solveKey=:default
)
  #  
  v = getVariable(dfg, vlbl)
  # vt = getVariableType(v)
  M = getManifold(v)
  e0 = Identity(M)
  
  @show b_Cwp = getPPESuggested(dfg, vlbl, solveKey)
  p_T_w = inv(M, exp(M, e0, hat(M, e0, b_Cwp)))
  p_H_w = affine_matrix(M, p_T_w)
  
  p_P1 = p_H_w * SA[w_BBo.origin...; 1.]
  p_P2 = p_H_w * SA[(w_BBo.origin+w_BBo.widths)...; 1.]
  
  GeoB.Rect3( GeoB.Point3(p_P1[1:3]...), GeoB.Point3((p_P2-p_P1)[1:3]...) )
end

"""
    $SIGNATURES

Is point p inside the HyperRectangle.
"""
function inside(
  hr::GeoB.HyperRectangle, 
  p::GeoB.Point
)
  #
  _p0,_p1 = GeoB.minmax(p, hr.origin, hr.origin + hr.widths)
  _p0 == hr.origin && hr.widths == (_p1-_p0)
end

"""
    $SIGNATURES

Create a new subcloud containing the portion of the full point cloud that is inside the mask.
"""
function getSubcloud(
  pc_full::PointCloud, # the full point cloud
  mask                 # starting out with GeometryBasics.Rect3
)
  objmask = map(s->_PCL.inside(mask,GeoB.Point(s.data[1:3]...)), pc_full.points)
  spt = (s->s.data[1:3]).(pc_full.points[objmask])
  @cast pts[i,d] := spt[i][d]
  _PCL.PointCloud(pts)
end



#