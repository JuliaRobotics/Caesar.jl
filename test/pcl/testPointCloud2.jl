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

import Caesar._PCL: FieldMapper, createMapping, PointCloud, PointField, PCLPointCloud2, Header, asType, _PCL_POINTFIELD_FORMAT, FieldMapping, MsgFieldMap, FieldMatches


##
@testset "test Caesar._PCL.PCLPointCloud2 to Caesar._PCL.PointCloud converter." begin
##

testdatafile = joinpath( dirname(dirname(Pkg.pathof(Caesar))), "test", "testdata", "_PCLPointCloud2.bson")

# load presaved test data to test the coverter
BSON.@load testdatafile PointCloudRef PointCloudTest

show(PointCloudTest[1])

# test major unpacking / type conversion needed by ROS.sensor_msgs.pointcloud2
pc = Caesar._PCL.PointCloud(PointCloudTest[1])

@test pc.height == 1
@test pc.width == 493
@test pc.is_dense

@test_broken length(pc.points) == 493

for (i,pt) in enumerate(pc.points)
  @test isapprox( PointCloudRef[1][1,:], pt.data; atol=1e-6)
end

# 231	    fromPCLPointCloud2 (msg, cloud, field_map);
# (gdb) print field_map
# $7 = std::vector of length 1, capacity 4 = {{serialized_offset = 0, 
#     struct_offset = 0, size = 12}}

# (gdb) print cloud.points[0]
# $43 = {<pcl::_PointXYZ> = {{data = {-0, 0, 0, 1}, {x = -0, y = 0, 
#         z = 0}}}, <No data fields>}
# (gdb) print sizeof(cloud.points[0])
# $44 = 16

##
end

#