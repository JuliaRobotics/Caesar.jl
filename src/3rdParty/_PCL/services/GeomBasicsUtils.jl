# Utils based on GeometryBasics





"""
    $SIGNATURES

Create a new subcloud containing the portion of the full point cloud that is inside the mask.
"""
function getSubcloud(
  pc_full::PointCloud,       # the full point cloud
  mask::AbstractBoundingBox; # starting out with GeometryBasics.Rect3
  minrange::Real=0, 
  maxrange::Real=999
)
  objmask = map(s->inside(mask, s.data[1:3]), pc_full.points)
  spt = (s->s.data[1:3]).(pc_full.points[objmask])
  if 0 == length(spt)
    return _PCL.PointCloud()
  end
  spt_ = _filterMinRange(spt, minrange, maxrange)
  @cast pts[i,d] := spt_[i][d]
  _PCL.PointCloud(pts)
end



#