# Utils based on GeometryBasics


"""
    $SIGNATURES

Is point p inside the HyperRectangle.
"""
function inside(hr::GeoB.HyperRectangle, p::GeoB.Point)
  _p0,_p1 = GeoB.minmax(p, hr.origin, hr.origin + hr.widths)
  _p0 == hr.origin && hr.widths == (_p1-_p0)
end

"""
    $SIGNATURES

Create a new subcloud containing the portion of the full point cloud that is inside the mask.
"""
function getSubcloud(
  pc_full, # the full pose variable point cloud as Type{<:TBD}
  mask # starting out with GeometryBasics.Rect3
)
  #
  objmask = map(s->_PCL.inside(mask,GeoB.Point(s.data[1:3]...)), pc_full.points)
  _PCL.PointCloud(pc_full.points[objmask])
end
