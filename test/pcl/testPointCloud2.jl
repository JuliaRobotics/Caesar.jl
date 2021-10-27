# requires python3 and rospy to be installed


# ENV["PYTHON"] = "/usr/bin/python3"
# Pkg.build("PyCall")
# using PyCall
# using RobotOS
using Colors
using Caesar
using BSON

using Test
using Pkg

##
@testset "test Caesar._PCL.PCLPointCloud2 to Caesar._PCL.PointCloud converter." begin
##

testdatafile = joinpath( dirname(dirname(Pkg.pathof(Caesar))), "test", "testdata", "_PCLPointCloud2.bson")

# load presaved test data to test the coverter
BSON.@load testdatafile PointCloudRef PointCloudTest

show(PointCloudTest[1])

pc = PointCloud(PointCloudTest[1])

@test pc.height == 1
@test pc.width == 493
@test pc.is_dense

@test_broken length(pc.points) == 493

for (i,pt) in enumerate(pc.points)
  @test isapprox( PointCloudRef[1][1,:], pt.data; atol=1e-6)
end

##
end

#