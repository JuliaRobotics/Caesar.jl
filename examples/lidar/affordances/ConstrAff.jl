
using Revise
using Caesar, Arena
const _PCL = Caesar._PCL

import Rotations as _Rot
import GeometryBasics as GeoB
using TensorCast
using Manifolds
using StaticArrays
using LasIO, FileIO
using MultivariateStats

##


# local copy version (if you have the file)
ffg = loadDFG("/tmp/caesar/X04_kstj.tar.gz")
# loadDFG("/tmp/caesar/X04_kstj_x50_x149.tar.gz")
# loadDFG("/tmp/caesar/X04_kstj_c1_x50_x70_json_.tar.gz")
sfg  = buildSubgraph(ffg, [Symbol("x$i") for i in 50:399], 1)
# assume only one prior for this dev example
DFG.addFactor!(sfg, [:x50], getFactor(ffg, lsfPriors(ffg)[1]))


##


# attach blob store to the in memory graph
storeDir = "/tmp/caesar/blobstore"
datastore = FolderStore{Vector{UInt8}}(:default_folder_store, storeDir)
addBlobStore!(sfg, datastore)


# # another precompile
# @time IIF.solveGraphParametric!(buildSubgraph(sfg, sortDFG(ls(sfg))[1:2], 1));
# # after compile run this should talk around 0.2s to solve, strong emphasis to use in-place and gradient operations for faster solves
# @time IIF.solveGraphParametric!(sfg);



# load prior model data

pc_modelprior = _PCL.loadLAS(joinpath(homedir(),"data/modelpriors/blueprint_slice.las"))
# plotPointCloud(pc_modelprior)

## Do with OAS factors

sfg_ = deepcopy(sfg)

getSolverParams(sfg_).graphinit=false

Mr = SpecialOrthogonal(3)
R0 = SMatrix{3,3}(diagm([1;1;1.]))

# manual object detection in the world frame
# first railing with floor-corner
w_BBo_01 = _PCL.OrientedBoundingBox( 
  [0;0;0.], 
  [6.2,4.0,3.], 
  [-2.5,-3.8,-1.0],  
  exp(Mr, R0, hat(Mr, R0, [0;0;-0.15]))
)
w_BBo_01b = _PCL.OrientedBoundingBox( 
  [0;0;0.], 
  [8.0,3.5,3.], 
  [-5.5,1.0,-1.0],  
  exp(Mr, R0, hat(Mr, R0, [0;0;-0.9]))
)

# floor
w_BBo_02 = _PCL.OrientedBoundingBox( 
  [0;0;0.], 
  [3.7,10,1.5] , 
  [4.2,-3.5,-1.0],  
  exp(Mr, R0, hat(Mr, R0, [0;0;-0.15]))
)
# stairway
w_BBo_03 = _PCL.OrientedBoundingBox( 
  [0;0;0.], 
  [3.7,6.5,4.0], 
  [0.5,3.5,-1.0],  
  exp(Mr, R0, hat(Mr, R0, [0;0;-0.15]))
)
# 2.5D floor plan
w_BBo_04 = _PCL.OrientedBoundingBox( 
  [0;0;0.], 
  [15,22,0.8], 
  [-5,-3.8,0.5],  
  exp(Mr, R0, hat(Mr, R0, [0;0;-0.142]))
)
w_BBo_05 = _PCL.OrientedBoundingBox( 
  [0;0;0.], 
  [4,5,4.], 
  [5,-5,-1.0],  
  exp(Mr, R0, hat(Mr, R0, [0;0;-0.15]))
)

w_BBo_06 = _PCL.OrientedBoundingBox( 
  [0;0;0.], 
  [4,8,7.], 
  [17,-2,-3.5],  
  exp(Mr, R0, hat(Mr, R0, [0;0;-0.15]))
)

w_BBo_07 = _PCL.OrientedBoundingBox( 
  [0;0;0.], 
  [10.5,8.5,8.], 
  [13.0,8.5,-3.5],  
  exp(Mr, R0, hat(Mr, R0, [0;0;-0.14]))
)

w_BBo_08 = _PCL.OrientedBoundingBox( 
  [0;0;0.], 
  [13,4.5,7.0], 
  [-1.0,13.0,-1.0],  
  exp(Mr, R0, hat(Mr, R0, [0;0;-0.14]))
)

w_BBo_09 = _PCL.OrientedBoundingBox( 
  [0;0;0.], 
  [10,10,8.0], 
  [-16.0,-15.0,-4.5],  
  exp(Mr, R0, hat(Mr, R0, [0;0;-0.14]))
)

w_BBo_10 = _PCL.OrientedBoundingBox( 
  [0;0;0.], 
  [12,8,5.0], 
  [-5.0,-18.0,-4.5],  
  # [11,8,5.0], 
  # [-4.0,-18.0,-4.5],  
  exp(Mr, R0, hat(Mr, R0, [0;0;-0.14]))
)


##

# filter possible pose keyframes having good sight of object affordance
vlbsBB01 = _PCL.findObjectVariablesFromWorld(sfg_, w_BBo_01; solveKey=:parametric, limit=10, minpoints=5000)
vlbsBB01b= _PCL.findObjectVariablesFromWorld(sfg_, w_BBo_01b; solveKey=:parametric, limit=99, minpoints=10000)
vlbsBB02 = _PCL.findObjectVariablesFromWorld(sfg_, w_BBo_02; solveKey=:parametric, limit=5, minpoints=2000)
vlbsBB03 = _PCL.findObjectVariablesFromWorld(sfg_, w_BBo_03; solveKey=:parametric, limit=20, minpoints=1000)
vlbsBB04 = _PCL.findObjectVariablesFromWorld(sfg_, w_BBo_04; solveKey=:parametric, limit=10, minpoints=7500)
vlbsBB05 = _PCL.findObjectVariablesFromWorld(sfg_, w_BBo_05; solveKey=:parametric, limit=20, minpoints=3000)
vlbsBB06 = _PCL.findObjectVariablesFromWorld(sfg_, w_BBo_06; solveKey=:parametric, limit=20, minpoints=1000)
vlbsBB07 = _PCL.findObjectVariablesFromWorld(sfg_, w_BBo_07; solveKey=:parametric, limit=20, minpoints=600)
vlbsBB08 = _PCL.findObjectVariablesFromWorld(sfg_, w_BBo_08; solveKey=:parametric, limit=20, minpoints=500)
vlbsBB09 = _PCL.findObjectVariablesFromWorld(sfg_, w_BBo_09; solveKey=:parametric, limit=20, minpoints=300)
vlbsBB10 = _PCL.findObjectVariablesFromWorld(sfg_, w_BBo_10; solveKey=:parametric, limit=20, minpoints=700)


## see whats going on

# f = Figure(resolution = (1920, 1080), backgroundcolor = RGB(37/255, 38/255, 40/255))
# set_theme!(backgroundcolor = RGB(37/255, 38/255, 40/255)) # :black
pl_tup = plotGraphPointClouds(
  sfg_; 
  solveKey=:parametric, 
  stride=100,
  varList = sortDFG(listVariables(sfg_)), #[180:end],
)
pl = pl_tup[1]

plotBoundingBox!(pl.axis, w_BBo_01)
plotBoundingBox!(pl.axis, w_BBo_01b)
# plotBoundingBox!(pl.axis, w_BBo_02)
plotBoundingBox!(pl.axis, w_BBo_03)
# plotBoundingBox!(pl.axis, w_BBo_04; color=:magenta)
plotBoundingBox!(pl.axis, w_BBo_05)
plotBoundingBox!(pl.axis, w_BBo_06)
plotBoundingBox!(pl.axis, w_BBo_07)
plotBoundingBox!(pl.axis, w_BBo_08)
plotBoundingBox!(pl.axis, w_BBo_09)
plotBoundingBox!(pl.axis, w_BBo_10)


pl

##

Caesar.generateObjectAffordanceFromWorld!(
  sfg_,
  :w_BBo_01,
  # [:x50; :x52; :x54; :x56;], 
  [:x50; :x52; :x54; :x55; :x57;],
  # vlbsBB01[1:4],
  # vlbsBB01,
  w_BBo_01;
  solveKey = :parametric
)

Caesar.generateObjectAffordanceFromWorld!(
  sfg_,
  :w_BBo_01b,
  [:x60; :x63; :x65; :x67; :x69],
  # vlbsBB01[1:4],
  # vlbsBB01,
  w_BBo_01b;
  solveKey = :parametric
)

Caesar.generateObjectAffordanceFromWorld!(
  sfg_,
  :w_BBo_03,
  # [:x66; :x67; :x68; :x69], # vlbsBB03,
  # intersect([:x66; :x68; :x69; :x71; :x73; :x75], ls(sfg_)), 
  intersect(vlbsBB03, ls(sfg_)),
  w_BBo_03;
  solveKey = :parametric
)


# Caesar.generateObjectAffordanceFromWorld!(
#   sfg_,
#   :w_BBo_04,
#   # vlbsBB04,
#   [:x50;:x54;:x58;:x63;:x67],
#   w_BBo_04;
#   solveKey = :parametric,
#   # modelprior=pc_modelprior
# )


Caesar.generateObjectAffordanceFromWorld!(
  sfg_,
  :w_BBo_05,
  # [:x63,:x64,:x66,:x67], #vlbsBB05,
  intersect([:x63; :x64; :x66; :x67; :x89; :x91; :x93; :x95; :x96; :x97], ls(sfg_)), 
  w_BBo_05;
  solveKey = :parametric
)

Caesar.generateObjectAffordanceFromWorld!(
  sfg_,
  :w_BBo_06a,
  # intersect(vlbsBB06, ls(sfg_)),
  intersect([Symbol("x",i) for i in 131:138], ls(sfg_)), 
  w_BBo_06;
  solveKey = :parametric
)
Caesar.generateObjectAffordanceFromWorld!(
  sfg_,
  :w_BBo_06b,
  # intersect(vlbsBB06, ls(sfg_)),
  intersect([Symbol("x",i) for i in 260:273], ls(sfg_)), 
  w_BBo_06;
  solveKey = :parametric
)

Caesar.generateObjectAffordanceFromWorld!(
  sfg_,
  :w_BBo_07,
  intersect(vlbsBB07, ls(sfg_)),
  # intersect([Symbol("x",i) for i in 133:143], ls(sfg_)), 
  w_BBo_07;
  solveKey = :parametric
)

Caesar.generateObjectAffordanceFromWorld!(
  sfg_,
  :w_BBo_07_200,
  intersect(vlbsBB07_200, ls(sfg_)),
  # intersect([Symbol("x",i) for i in 242:262], ls(sfg_)), 
  w_BBo_07;
  solveKey = :parametric
)

Caesar.generateObjectAffordanceFromWorld!(
  sfg_,
  :w_BBo_08a,
  # intersect(vlbsBB08, ls(sfg_)),
  intersect([Symbol("x",i) for i in 97:110], ls(sfg_)), 
  w_BBo_08;
  solveKey = :parametric
)
Caesar.generateObjectAffordanceFromWorld!(
  sfg_,
  :w_BBo_08b,
  # intersect(vlbsBB08, ls(sfg_)),
  intersect([Symbol("x",i) for i in 289:299], ls(sfg_)), 
  w_BBo_08;
  solveKey = :parametric
)

Caesar.generateObjectAffordanceFromWorld!(
  sfg_,
  :w_BBo_09a,
  # intersect(vlbsBB09, ls(sfg_)),
  intersect([Symbol("x",i) for i in 77:93], ls(sfg_)), 
  w_BBo_09;
  solveKey = :parametric
)

Caesar.generateObjectAffordanceFromWorld!(
  sfg_,
  :w_BBo_10a,
  # intersect(vlbsBB10, ls(sfg_)),
  intersect([[Symbol("x",i) for i in 99:106]; :x62], ls(sfg_)), 
  w_BBo_10;
  solveKey = :parametric
)
Caesar.generateObjectAffordanceFromWorld!(
  sfg_,
  :w_BBo_10b,
  # intersect(vlbsBB10, ls(sfg_)),
  intersect([Symbol("x",i) for i in 107:112], ls(sfg_)), 
  w_BBo_10;
  solveKey = :parametric
)


## now look at fine alignment within on OAS factor caches

foaslb = ls(sfg_, :w_BBo_05)[1]
oas = getFactorType(sfg_, foaslb)
foaslb

##
Caesar.makePointCloudObjectAffordance(sfg_, foaslb; align=:coarse) |> plotPointCloud
Caesar.makePointCloudObjectAffordance(sfg_, foaslb; align=:fine) |> plotPointCloud
Caesar.assembleObjectCache(sfg_, foaslb) |> plotPointCloud


## check obj addition  


Caesar.protoObjectCheck!(
  sfg_,
  w_BBo_05;
  # align = :coarse, # :fine,
  # varList = vlbsBB07_200,
  # varList = [Symbol("x",i) for i in 107:112],
  # minpoints=1000, limit=20, selection=:biggest,
  varList = _PCL.findObjectVariablesFromWorld(
    sfg_, 
    w_BBo_05; 
    limit=20, minpoints=3000,
    varList = sortDFG(ls(sfg_))[205:end],
  ) ,
) |> plotPointCloud
# varList = [:x50; :x52; :x54; :x55; :x57;], ## BB01 
# varList = [:x66; :x68; :x71; :x73; :x75],  ## BB03
# varList = [:x63; :x64; :x66; :x67; :x89; :x91; :x93; :x95; :x96; :x97;], #  :x140; :x141; :x144; :x145], #BB05,
# varList = [:x131; :x135; :x136; :x137; :x138]  ## BB06
# varList = vlbsBB06[2:2:20],

##

IIF.solveGraphParametric!(sfg_) 



## 
# vlbsBB04
pl_tup = plotGraphPointClouds(
  sfg_; 
  varList = sortDFG(listVariables(sfg_)),
  solveKey=:parametric,
  stride=50,
)
pl = pl_tup[1]

plotBoundingBox!(pl.axis, w_BBo_01)
plotBoundingBox!(pl.axis, w_BBo_01b)
# plotBoundingBox!(pl.axis, w_BBo_02)
plotBoundingBox!(pl.axis, w_BBo_03)
# plotBoundingBox!(pl.axis, w_BBo_04; color=:magenta)
plotBoundingBox!(pl.axis, w_BBo_05)
plotBoundingBox!(pl.axis, w_BBo_06)
plotBoundingBox!(pl.axis, w_BBo_07)
plotBoundingBox!(pl.axis, w_BBo_08)
plotBoundingBox!(pl.axis, w_BBo_09)
plotBoundingBox!(pl.axis, w_BBo_10)

pl

## add plot for prior



## random objects

vlbsBB07_200 = _PCL.findObjectVariablesFromWorld(
  sfg_, 
  w_BBo_07; 
  varList=sortDFG(ls(sfg_))[205:225],
  solveKey=:parametric, 
  limit=20, 
  minpoints=500
)


## Check alignment logic

cache = IIF._getCCW(sfg_, ls(sfg_, :x54w_BBo_01f1)[1]).dummyCache

o_Ts_p = cache.o_Ts_p
p_SCs  = cache.p_SCs
# align iteratively
oo_Ts_p = _PCL.alignPointCloudsLOOIters!(
  o_Ts_p, 
  p_SCs, 
  false, 3
)

## ================================
## debug object frame
## ================================

M = getManifold(Pose3)
e0 = ArrayPartition(MVector(0,0,0.), MMatrix{3,3}(1,0,0,0,1,0,0,0,1.))

foaslb = ls(sfg_, :w_BBo_01)[1]
oas = getFactorType(sfg_, foaslb)
cache = IIF._getCCW(sfg_, foaslb).dummyCache;

##

# lhat_SC, ohat_Ts_li = _PCL.mergePointCloudsWithTransforms(oas.lhat_Ts_p, cache.p_SCs);
# plotPointCloud(lhat_SC)

##

lhat_SCs = [_PCL.apply(M, oas.lhat_Ts_p[n], cache.p_SCs[n]) for n in 1:4];
lhat_Ts_lhat = [e0,e0,e0,e0]

lhat_SC, ohat_Ts_lhat = _PCL.mergePointCloudsWithTransforms(lhat_Ts_lhat, lhat_SCs);

plotPointCloud(lhat_SC)


## ==== THREE STEP OPTION ==== aligns around nearby ohat frame

ohat_SCs = [_PCL.apply(M, ohat_Ts_lhat[n], lhat_SCs[n]) for n in 1:4];
o_Ts_ohat = [e0,e0,e0,e0]

_PCL.alignPointCloudsLOOIters!(
  o_Ts_ohat,
  ohat_SCs
)
o_SC, oo_Ts_ohat = _PCL.mergePointCloudsWithTransforms(o_Ts_ohat, ohat_SCs);
plotPointCloud(o_SC)


## ==== TWO STEP OPTION ==== aligns around world frame origin

_PCL.alignPointCloudsLOOIters!(
  ohat_Ts_lhat,
  lhat_SCs;
  updateTloo=true
)
ohat_SC, o_Ts_lhat = _PCL.mergePointCloudsWithTransforms(ohat_Ts_lhat, lhat_SCs);
plotPointCloud(ohat_SC)


## ==== from coarse poses to object frame ====

ohat_Ts_p = [compose(M, ohat_Ts_lhat[i], oas.lhat_Ts_p[i]) for i in 1:4]

_PCL.mergePointCloudsWithTransforms(ohat_Ts_p, cache.p_SCs; flipXY=false)[1] |> plotPointCloud



## =====================================
## SANITY CHECK OAS and CACHE ALIGNMENT
## =====================================

M = getManifold(Pose3)
foaslb = ls(sfg_, :w_BBo_01)[1]
oas = getFactorType(sfg_, foaslb)
cache = IIF._getCCW(sfg_, foaslb).dummyCache
vlbs = getVariableOrder(sfg_[foaslb])[2:end]

oas.p_PC_blobIds

p_PCs = [_PCL.PointCloud(_PCL.getDataPointCloud(sfg_, vlbs[i], oas.p_PC_blobIds[i]; checkhash=false)) for i in 1:4]
p_SCs = [_PCL.getSubcloud(p_PCs[i], oas.p_BBos[i]) for i in 1:4]
o_Ts_p = [compose(M, cache.o_Ts_ohat[i], cache.ohat_Ts_p[i]) for i in 1:4]
# CHECK: hard override on ohat frame
  # ohat_Ts_p = [cache.ohat_Ts_p[i] for i in 1:4]
  # o_Ts_p = ohat_Ts_p
# CHECK: hard override on lhat frame
  # o_Ts_p = oas.lhat_Ts_p
o_SCs = [_PCL.apply(M, o_Ts_p[i], p_SCs[i]) for i in 1:4]

o_SC = deepcopy(o_SCs[1])
for i in 2:4
  _PCL.cat(o_SC, o_SCs[i]; reuse=true)
end

plotPointCloud(o_SC)


##