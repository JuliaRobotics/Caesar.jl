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

# import Caesar._PCL: FieldMapper, createMapping, PointCloud, PointField, PCLPointCloud2, Header, asType, _PCL_POINTFIELD_FORMAT, FieldMapping, MsgFieldMap, FieldMatches


##
@testset "test Caesar._PCL.PCLPointCloud2 to Caesar._PCL.PointCloud converter." begin
##

# testdatafile = joinpath( pkgdir(Caesar), "test", "testdata", "_PCLPointCloud2.bson")
# load presaved test data to test the coverter
# BSON.@load testdatafile PointCloudRef PointCloudTest

## build PCLPointCloud2 to be converted

datafile = joinpath( pkgdir(Caesar), "test", "testdata", "_PCLPointCloud2_15776.dat")
fid = open(datafile,"r")
data = read(fid)
close(fid)

##

pc2 = Caesar._PCL.PCLPointCloud2(;
  header = Caesar._PCL.Header(;
    seq = 92147,
    stamp = 0x0005b24647c4af10,
    frame_id = "velodyne"
  ),
  height = 1,
  width = 493,
  fields = [
    Caesar._PCL.PointField("x", 0x00000000, Caesar._PCL._PCL_FLOAT32, 0x00000001),
    Caesar._PCL.PointField("y", 0x00000004, Caesar._PCL._PCL_FLOAT32, 0x00000001),
    Caesar._PCL.PointField("z", 0x00000008, Caesar._PCL._PCL_FLOAT32, 0x00000001),
    Caesar._PCL.PointField("intensity", 0x00000010, Caesar._PCL._PCL_FLOAT32, 0x00000001)
  ],
  point_step = 32,
  row_step = 15776,
  is_dense = 1,
  data
)


##

show(pc2)

# test major unpacking / type conversion needed by ROS.sensor_msgs.pointcloud2
pc = Caesar._PCL.PointCloud(pc2)

show(pc)

@test pc.height == 1
@test pc.width == 493
@test pc.is_dense

@test length(pc.points) == 493

##

# truncated copy of reference data on good to 1e-3
@test isapprox( [-0,0,0],              pc.points[1].data[1:3], atol=5e-3)
@test isapprox( [-1.56486,1.09851,0],  pc.points[2].data[1:3], atol=5e-3)
@test isapprox( [-793.383,556.945,0],  pc.points[3].data[1:3], atol=5e-3)
@test isapprox( [-0,0,0],              pc.points[4].data[1:3], atol=5e-3)
@test isapprox( [-1.56148,1.10331,0],  pc.points[5].data[1:3], atol=5e-3)
@test isapprox( [-788.548,557.169,0],  pc.points[6].data[1:3], atol=5e-3)
@test isapprox( [-790.109,558.273,0],  pc.points[7].data[1:3], atol=5e-3)
@test isapprox( [-791.671,559.376,0],  pc.points[8].data[1:3], atol=5e-3)
@test isapprox( [-793.232,560.479,0],  pc.points[9].data[1:3], atol=5e-3)
@test isapprox( [-794.794,561.583,0],  pc.points[10].data[1:3], atol=5e-3)


# for (i,pt) in enumerate(pc.points)
#   @test isapprox( PointCloudRef[1][i,1:3], pt.data[1:3]; atol=1e-6)
# end


##
end


@testset "PandarXT test point cloud conversion test" begin
##

@warn "TODO, see testdata/_pandar_PCLPointCloud2.jldat which via `Serialization.serialize` of a `Caesar._PCL.PCLPointCloud2` object, at JL 1.7.3, CJL v0.13.1+" 
# pc2.fields
#   6-element Vector{Caesar._PCL.PointField}:
#  Caesar._PCL.PointField("x", 0x00000000, Caesar._PCL._PCL_FLOAT32, 0x00000001)
#  Caesar._PCL.PointField("y", 0x00000004, Caesar._PCL._PCL_FLOAT32, 0x00000001)
#  Caesar._PCL.PointField("z", 0x00000008, Caesar._PCL._PCL_FLOAT32, 0x00000001)
#  Caesar._PCL.PointField("intensity", 0x00000010, Caesar._PCL._PCL_FLOAT32, 0x00000001)
#  Caesar._PCL.PointField("timestamp", 0x00000018, Caesar._PCL._PCL_FLOAT64, 0x00000001)
#  Caesar._PCL.PointField("ring", 0x00000020, Caesar._PCL._PCL_UINT16, 0x00000001)


##
end



#