# requires python3 and rospy to be installed


# ENV["PYTHON"] = "/usr/bin/python3"
# Pkg.build("PyCall")
# using PyCall
# using RobotOS

using Colors
using Caesar
using BSON
using Serialization
using FixedPointNumbers

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

@info "test `apply` transform SpecialEuclidean(2) to 2D pointcloud."

M = SpecialEuclidean(2)
rPc = ArrayPartition([1.;0], [1 0; 0 1.])

pc_ = Caesar._PCL.apply(M, rPc, pc)


@test isapprox( [1-0,0,0],              pc_.points[1].data[1:3], atol=5e-3)
@test isapprox( [1-1.56486,1.09851,0],  pc_.points[2].data[1:3], atol=5e-3)
@test isapprox( [1-793.383,556.945,0],  pc_.points[3].data[1:3], atol=5e-3)
@test isapprox( [1-0,0,0],              pc_.points[4].data[1:3], atol=5e-3)
@test isapprox( [1-1.56148,1.10331,0],  pc_.points[5].data[1:3], atol=5e-3)
@test isapprox( [1-788.548,557.169,0],  pc_.points[6].data[1:3], atol=5e-3)
@test isapprox( [1-790.109,558.273,0],  pc_.points[7].data[1:3], atol=5e-3)
@test isapprox( [1-791.671,559.376,0],  pc_.points[8].data[1:3], atol=5e-3)
@test isapprox( [1-793.232,560.479,0],  pc_.points[9].data[1:3], atol=5e-3)
@test isapprox( [1-794.794,561.583,0],  pc_.points[10].data[1:3], atol=5e-3)


##
end


@testset "PandarXT test point cloud conversion test" begin
##

# Alternative approach, see more hardcoded test data example (only .data writen to binary) for _PCLPointCloud2_15776.dat"
@info "Loading testdata/_pandar_PCLPointCloud2.jldat which via `Serialization.serialize` of a `Caesar._PCL.PCLPointCloud2` object, at JL 1.7.3, CJL v0.13.1+" 
datafile = joinpath( pkgdir(Caesar), "test", "testdata", "_pandar_PCLPointCloud2.jldat")
pc2 = Serialization.deserialize(datafile)
# pc2.fields
#   6-element Vector{Caesar._PCL.PointField}:
#  Caesar._PCL.PointField("x", 0x00000000, Caesar._PCL._PCL_FLOAT32, 0x00000001)
#  Caesar._PCL.PointField("y", 0x00000004, Caesar._PCL._PCL_FLOAT32, 0x00000001)
#  Caesar._PCL.PointField("z", 0x00000008, Caesar._PCL._PCL_FLOAT32, 0x00000001)
#  Caesar._PCL.PointField("intensity", 0x00000010, Caesar._PCL._PCL_FLOAT32, 0x00000001)
#  Caesar._PCL.PointField("timestamp", 0x00000018, Caesar._PCL._PCL_FLOAT64, 0x00000001)
#  Caesar._PCL.PointField("ring", 0x00000020, Caesar._PCL._PCL_UINT16, 0x00000001)

##

pc = Caesar._PCL.PointCloud(pc2)

@test UInt32(1) === pc.height
@test UInt32(58015) === pc.width
@test 58015 === length(pc.points)
@test pc.points[1] isa Caesar._PCL.PointXYZ{RGBA{FixedPointNumbers.N0f8}, Float32}
@test pc.is_dense
@test pc.header.seq === UInt32(24285)

# check actual points
@test isapprox( Float32[0.009075656, 3.714255, 0.99023366, 0.0], pc.points[1].data[1:4], atol=5e-3)
@test isapprox( Float32[0.008430854, 3.715781, 0.92004687, 0.0], pc.points[2].data[1:4], atol=5e-3)
@test isapprox( Float32[0.0084575275, 3.727537, 0.8534482, 0.0], pc.points[3].data[1:4], atol=5e-3)
@test isapprox( Float32[0.008463516, 3.7301762, 0.78552955, 0.0], pc.points[4].data[1:4], atol=5e-3)
@test isapprox( Float32[0.0032057164, 1.3119546, 0.25231096, 0.0], pc.points[5].data[1:4], atol=5e-3)

## convert back to PCLPointCloud2

@info "test back conversion from _PCL.PointCloud to _PCL.PCLPointCloud2, and again _PCL.PointCloud"

_pc2 = Caesar._PCL.PCLPointCloud2(pc)
_pc = Caesar._PCL.PointCloud(_pc2)

@test UInt32(1) === _pc.height
@test UInt32(58015) === _pc.width
@test 58015 === length(_pc.points)
@test _pc.points[1] isa Caesar._PCL.PointXYZ{RGBA{FixedPointNumbers.N0f8}, Float32}
@test _pc.is_dense
@test _pc.header.seq === UInt32(24285)

# check actual points
@test isapprox( Float32[0.009075656, 3.714255, 0.99023366, 0.0], _pc.points[1].data[1:4], atol=5e-3)
@test isapprox( Float32[0.008430854, 3.715781, 0.92004687, 0.0], _pc.points[2].data[1:4], atol=5e-3)
@test isapprox( Float32[0.0084575275, 3.727537, 0.8534482, 0.0], _pc.points[3].data[1:4], atol=5e-3)
@test isapprox( Float32[0.008463516, 3.7301762, 0.78552955, 0.0], _pc.points[4].data[1:4], atol=5e-3)
@test isapprox( Float32[0.0032057164, 1.3119546, 0.25231096, 0.0], _pc.points[5].data[1:4], atol=5e-3)


## test SpecialEuclidean(3) transform application

M = SpecialEuclidean(3)
rPc = ArrayPartition([-1;0;10.], [1 0 0; 0 1 0; 0 0 1.])

pc_3D = Caesar._PCL.apply(M, rPc, pc)


@test UInt32(1) === pc_3D.height
@test UInt32(58015) === pc_3D.width
@test 58015 === length(pc_3D.points)
@test pc_3D.points[1] isa Caesar._PCL.PointXYZ{RGBA{FixedPointNumbers.N0f8}, Float32}
@test pc_3D.is_dense
@test pc_3D.header.seq === UInt32(24285)

@test isapprox( Float32[-1+0.009075656, 3.714255,  10+ 0.99023366, 0.0], pc_3D.points[1].data[1:4], atol=5e-3)
@test isapprox( Float32[-1+0.008430854, 3.715781,  10+ 0.92004687, 0.0], pc_3D.points[2].data[1:4], atol=5e-3)
@test isapprox( Float32[-1+0.0084575275, 3.727537, 10+ 0.8534482, 0.0], pc_3D.points[3].data[1:4], atol=5e-3)
@test isapprox( Float32[-1+0.008463516, 3.7301762, 10+ 0.78552955, 0.0], pc_3D.points[4].data[1:4], atol=5e-3)
@test isapprox( Float32[-1+0.0032057164, 1.3119546,10+ 0.25231096, 0.0], pc_3D.points[5].data[1:4], atol=5e-3)


##
end



#