# PoC on stashing for ScatterAlignPose2

using Test
using Caesar
using Images
using Manifolds
import Rotations as _Rot


##

@testset "test ScatterAlignPose2 with stashed deserialization" begin
## Build a factor graph with ScatterAlignPose2 factors and set for stashing serialization


x = -10:0.1:10;
y = -10:0.1:10;

σ = 0.1

Σ = Diagonal([σ;σ])
g = (x,y)->pdf(MvNormal([3.;0],Σ),[x;y]) + pdf(MvNormal([8.;0.0],4*Σ),[x;y]) + pdf(MvNormal([0;5.0],Σ),[x;y])

bIM1 = zeros(length(x),length(y))
bIM2 = zeros(length(x),length(y))

oT = [2.; 0]
oΨ =  pi/6

M = SpecialEuclidean(2)
e0 = identity_element(M)
pCq = [oT;oΨ]
qGp = inv(M, exp(M, e0, hat(M, e0, pCq)))
qTp = affine_matrix(M, qGp )

qCp = vee(M, e0, log(M, e0, qGp))


for (i,x_) in enumerate(x), (j,y_) in enumerate(y)
  bIM1[i,j] = g(x_,y_)
  v = qTp*[x_;y_;1.0]
  _x_, _y_ = v[1], v[2]
  bIM2[i,j] = g(_x_, _y_)
end


##

fg = initfg()
getSolverParams(fg).inflateCycles=1

##

# getSolverParams(fg).logpath = pwd()
storeDir = joinLogPath(fg,"data")
mkpath(storeDir)
datastore = FolderStore{Vector{UInt8}}(:test_stashing_store, storeDir) 
addBlobStore!(fg, datastore)

##

addVariable!(fg, :x0, Pose2)
addVariable!(fg, :x1, Pose2)


##

sap = ScatterAlignPose2(bIM1, bIM2, (x,y); 
                        sample_count=100, bw=1.0, 
                        cvt=(im)->im, 
                        useStashing=false)
                        # dataEntry_cloud1=string(:hgd_stash_x0),
                        # dataEntry_cloud2=string(:hgd_stash_x1),
                        # dataStoreHint = string(:test_stashing_store) )
#

## must store the stashed data entry blobs before the preambleCache function runs on addFactor

db = Vector{UInt8}( convert(String, sap.align.cloud1) )
de0, = addData!(fg, :test_stashing_store, :x0, :hgd_stash_x0, db, mimeType="application/json/octet-stream")

db = Vector{UInt8}( convert(String,sap.align.cloud2) )
de1, = addData!(fg, :test_stashing_store, :x1, :hgd_stash_x1, db, mimeType="application/json/octet-stream")

##

sap = ScatterAlignPose2(bIM1, bIM2, (x,y); 
                        sample_count=100, bw=1.0, 
                        cvt=(im)->im, 
                        useStashing=false, 
                        dataEntry_cloud1=string(de0.id),
                        dataEntry_cloud2=string(de1.id),
                        dataStoreHint = string(:test_stashing_store) )
#


##

de, db_ = getData(fg, :x0, :hgd_stash_x0)

# myData = JSON3.read(String(db), PackedHeatmapGridDensity)

# JSON.parse(String(db))["_type"] |> DFG.getTypeFromSerializationModule

# convert(SamplableBelief, String(db))


##

addFactor!(fg, [:x0;], PriorPose2(MvNormal([0;0;0.],[0.01;0.01;0.01])))
f = addFactor!(fg, [:x0;:x1], sap, inflation=0.0)


## Test packFactor with useStashing


pf = DFG.packFactor(fg, f)


## save the factor graph

filepath = joinLogPath(fg, "selftest")
saveDFG(filepath, fg)

## load the factor graph using unstash 

fg_ = initfg()
datastore_ = FolderStore{Vector{UInt8}}(:test_stashing_store, storeDir) 
addBlobStore!(fg_, datastore_)

loadDFG!(fg_, filepath)


## confirm that the right factor graph has be reconstituted

@test isapprox( getFactorType(fg, :x0x1f1).align.cloud1.data,getFactorType(fg_, :x0x1f1).align.cloud1.data )
@test isapprox( getFactorType(fg, :x0x1f1).align.cloud2.data,getFactorType(fg_, :x0x1f1).align.cloud2.data )

##


@test_broken isapprox(
  getPoints(getFactorType(fg, :x0x1f1).align.cloud1.densityFnc)[1],
  getPoints(getFactorType(fg_, :x0x1f1).align.cloud1.densityFnc)[1]
)



##
end

#