# quick dev on building check point inside hyper rectangle function

using Test
# import GeometryBasics as GeoB
import Caesar._PCL as _PCL

##

@testset "test basic Rect3 bounding box logic" begin
##

p0 = GeoB.Point(0,0,0.)
p1 = GeoB.Point(1,1,1.)
p2 = GeoB.Point(0,0,3.)

hr = GeoB.Rect([p1,p2])

# function inside(hr::GeoB.HyperRectangle, p)
#   _p0,_p1 = GeoB.minmax(p, hr.origin, hr.origin + hr.widths)
#   _p0 == hr.origin && hr.widths == (_p1-_p0)
# end

@test _PCL.inside(hr, p1)
@test !_PCL.inside(hr, p0)
@test _PCL.inside(hr, p2)

##
end

#