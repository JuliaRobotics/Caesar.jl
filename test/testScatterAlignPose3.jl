
using Test
using Caesar
# import Caesar._PCL as _PCL
# using Downloads
# using DelimitedFiles
# using TensorCast
using Images
# using JSON

const _PCL = Caesar._PCL

##

@testset "test ScatterAlignPose3 on ICP" begin
##

## ~50mb download, blobs that are too big to always store in variables themselves
# lidar1_url = "https://github.com/JuliaRobotics/CaesarTestData.jl/raw/main/data/lidar/simpleICP/terrestrial_lidar1.xyz"
# lidar2_url = "https://github.com/JuliaRobotics/CaesarTestData.jl/raw/main/data/lidar/simpleICP/terrestrial_lidar2.xyz"
# io1 = PipeBuffer()
# io2 = PipeBuffer()
# Downloads.download(lidar1_url, io1)
# Downloads.download(lidar2_url, io2)

# X_fix = readdlm(io1)
# X_mov = readdlm(io2)

X_fix = randn(10000,3)
X_mov = randn(10000,3)

pcf = _PCL.PointCloud(X_fix)
pcm = _PCL.PointCloud(X_mov)

# @cast pts_f_[i][d] := X_fix[i,d]
# @cast pts_m_[i][d] := X_mov[i,d]

# pts_f = (s->[s...]).(pts_f_)
# pts_m = (s->[s...]).(pts_m_)


##

fg = initfg()

addVariable!(fg, :x0, Pose3)
addVariable!(fg, :x1, Pose3)

##

# this is how how you call the ICP version of SAP
sap = Caesar.ScatterAlignPose3(
  ManifoldKernelDensity;
  cloud1=pcf, 
  cloud2=pcm,
  sample_count=-1,
  useStashing=false,
)

## this line checks blob store access via preambleCache specifically for ScatterAlign which will internally access the blob store

f1 = addFactor!(fg, [:x0; :x1], sap; graphinit=false);

## make sure stuff is working before serialization
meas = sampleFactor(fg, getLabel(f1))


@warn "Testing of ICP based ScatterAlignPose3 serialization must still be done"
## serialize with stashing enabled, see docs here: https://juliarobotics.org/Caesar.jl/latest/concepts/stash_and_cache/

# pf = DFG.packFactor(fg, getFactor(fg, getLabel(f1)))
# jpf = JSON.json(pf)

# check that the massive point clouds are not stored in the packed factor object
# @test length(jpf) < 1100

## now confirm the solver deserialization can also work with the factor pulling data from the blob store 

# tfg = initfg()

# # use existing logpath from fg
# storeDir = "/tmp/caesar/localstore" # joinLogPath(fg,"data")

# datastore = FolderStore{Vector{UInt8}}(:default_folder_store, storeDir) 
# addBlobStore!(tfg, datastore)
# v0 = addVariable!(tfg, :x0, Pose3)
# v1 = addVariable!(tfg, :x1, Pose3)


##
end