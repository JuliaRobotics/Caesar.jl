# Utils based on GeometryBasics


"""
    $SIGNATURES

Is point p inside the HyperRectangle.
"""
function inside(hr::GeoB.HyperRectangle, p::GeomB.Point)
  _p0,_p1 = GeoB.minmax(p, hr.origin, hr.origin + hr.widths)
  _p0 == hr.origin && hr.widths == (_p1-_p0)
end