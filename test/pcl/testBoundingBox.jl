# quick dev on building check point inside hyper rectangle function

# using Revise
using Test
import Caesar._PCL as _PCL

##

@testset "test basic _PCL.AxisAlignedBoundingBox logic" begin
##

p0 = [0,0,0.] # GeoB.Point(
p1 = [0,0,1.] # GeoB.Point(
p1b = [0.1,0.1,1.1] # GeoB.Point(
p2 = [1,1,2.] # GeoB.Point(
p3 = [0,0,4.] # GeoB.Point(
p4 = [0.5;0.5;2]

hr = _PCL.AxisAlignedBoundingBox( p1,p2 )

@test _PCL.inside(hr, p1b)
@test !_PCL.inside(hr, p0)
@test !_PCL.inside(hr, p3)
@test _PCL.inside(hr, p4)

##
end

#