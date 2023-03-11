
using Test
using Caesar
# import Caesar._PCL as _PCL
using Downloads
using DelimitedFiles
using TensorCast
using Images
using JSON

##

@testset "test stashing in factor packing and unpacking" begin
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

@cast pts_f_[i][d] := X_fix[i,d]
@cast pts_m_[i][d] := X_mov[i,d]

pts_f = (s->[s...]).(pts_f_)
pts_m = (s->[s...]).(pts_m_)

# pcf = _PCL.PointCloud(X_fix)
# pcm = _PCL.PointCloud(X_mov)

##

bel_f = manikde!(Position3, pts_f; bw=[0.1;0.1;0.1])
bel_m = manikde!(Position3, pts_m; bw=[0.1;0.1;0.1])

##

fg = initfg()

storeDir = "/tmp/caesar/localstore" # joinLogPath(fg,"data")
mkpath(storeDir)
datastore = FolderStore{Vector{UInt8}}(:default_folder_store, storeDir) 
addBlobStore!(fg, datastore)

##

addVariable!(fg, :x0, Pose3)
addVariable!(fg, :x1, Pose3)

##
# addFactor!(fg, [:x0], MvNormal(zeros(6)))

##

data = packDistribution(bel_f)
deb0 = addData!(
  fg, 
  :default_folder_store, 
  :x0, 
  :pointcloud, 
  Vector{UInt8}(JSON.json( data )), 
  mimeType="application/json/octet-stream"  
)
data = packDistribution(bel_m)
deb1 = addData!(
  fg, 
  :default_folder_store, 
  :x1, 
  :pointcloud, 
  Vector{UInt8}(JSON.json( data )), 
  mimeType="application/json/octet-stream"  
)

##

sap = Caesar.ScatterAlignPose3(;
  cloud1=bel_f, 
  cloud2=bel_m,
  sample_count=100,
  useStashing=true,
  dataEntry_cloud1=string(deb0[1].id),
  dataEntry_cloud2=string(deb1[1].id),
)

## this line checks blob store access via preambleCache specifically for ScatterAlign which will internally access the blob store

f1 = addFactor!(fg, [:x0; :x1], sap; graphinit=false);

## make sure stuff is working before serialization
meas = sampleFactor(fg, getLabel(f1), 1)

## serialize with stashing enabled, see docs here: https://juliarobotics.org/Caesar.jl/latest/concepts/stash_and_cache/

pf = DFG.packFactor(fg, getFactor(fg, getLabel(f1)))

jpf = JSON.json(pf)

# check that the massive point clouds are not stored in the packed factor object
@test length(jpf) < 1500

## now confirm the solver deserialization can also work with the factor pulling data from the blob store 

tfg = initfg()

# use existing logpath from fg
storeDir = "/tmp/caesar/localstore" # joinLogPath(fg,"data")

datastore = FolderStore{Vector{UInt8}}(:default_folder_store, storeDir) 
addBlobStore!(tfg, datastore)
v0 = addVariable!(tfg, :x0, Pose3)
v1 = addVariable!(tfg, :x1, Pose3)

# To load from stashed blobs attached to variables, those variables need the associated dataEntries
Caesar.addDataEntry!(v0,deb0[1])
Caesar.addDataEntry!(v1,deb1[1])

# test unpacking of factor pulling data from blobstore during preambleCache
pf_ = Caesar.unpackFactor(tfg,pf)

## all up test via FileDFG

saveDFG("/tmp/caesar/test_stashed_data", fg)

fg_  = initfg()

# attach existing blob store before loading
storeDir = "/tmp/caesar/localstore" # joinLogPath(fg,"data")
datastore = FolderStore{Vector{UInt8}}(:default_folder_store, storeDir) 
addBlobStore!(fg_, datastore)

loadDFG!(fg_, "/tmp/caesar/test_stashed_data")

##
end