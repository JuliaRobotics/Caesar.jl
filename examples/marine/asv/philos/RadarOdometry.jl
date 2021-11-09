
using Distributed
# addprocs(4)

using Colors
using Caesar

using DistributedFactorGraphs.DocStringExtensions
using Dates
using JSON2
using BSON
using Serialization
using FixedPointNumbers
using StaticArrays
using ImageMagick, FileIO
using Images

using ImageDraw

##

## after the graph is saved it can be loaded and the datastores retrieved

dfg_datafolder = "/tmp/caesar/philos"

fg = loadDFG("$dfg_datafolder/dfg")

ds = FolderStore{Vector{UInt8}}(:radar, "$dfg_datafolder/data/radar")
addBlobStore!(fg, ds)

ds = FolderStore{Vector{UInt8}}(:gps_fix, "$dfg_datafolder/data/gps")
addBlobStore!(fg, ds)

# add if you want lidar also 
ds = FolderStore{Vector{UInt8}}(:lidar, "$dfg_datafolder/data/lidar")
addBlobStore!(fg, ds)


## visualize the radar data

if false
  imgs = map(x->Gray{N0f8}.(x), fetchDataImage.(fg, sortDFG(ls(fg)), :RADAR_IMG));
  writevideo("/tmp/caesar/philos/radar.mp4", imgs; fps=5, player="totem")
end


## For this example, lets only add a few transform factors between poses, say every 10th pose

# set case specific parameters
getSolverParams(fg).inflateCycles=1

# switch of all variables
setSolvable!.(fg, ls(fg), 0)

# select the pose variables to include in the solve
slv = [Symbol("x",i) for i in 0:10:30]
setSolvable!.(fg, slv, 1)

# add a PriorPose2
if 0==length(lsf(fg, tags=[:ORIGIN]))
  addFactor!(fg, [:x0;], PriorPose2(MvNormal([0;0;0.],0.01*[0;0;0.])), tags=[:ORIGIN;])
end

## load the point clouds and create the radar odometry factors

bw = 2.0
for (i,lb) in enumerate(slv[1:(end-1)])
  # Cloud 1, removing values near vehicle itself
  de,db = getData(fg, lb, :RADAR_SWEEP)
  sweep = deserialize(PipeBuffer(db)) # BSON.@load
  XY = map(pt->pt.data[1:2], sweep.points)
  XY_ = filter(pt->10 < norm(pt), XY)
  r1 = manikde!(getManifold(Point2), XY_, bw=[bw;bw])

  # Cloud2, removing values near vehicle itself
  lb_ = slv[i+1]
  de,db = getData(fg, lb_, :RADAR_SWEEP)
  sweep = deserialize(PipeBuffer(db)) # BSON.@load
  XY = map(pt->pt.data[1:2], sweep.points)
  XY_ = filter(pt->10 < norm(pt), XY)
  r2 = manikde!(getManifold(Point2), XY_, bw=[bw;bw])

  # create the radar alignment factor
  sap = ScatterAlignPose2(;hgd1=r1, hgd2=r2, bw=0.0001, sample_count=150)

  # add the new factor to the graph
  addFactor!(fg, [lb; lb_], sap, inflation=0.0, solvable=1, tags=[:RADAR_ODOMETRY])
end


## make a copy of this subgraph for debugging

fg_ = initfg()
getSolverParams(fg).inflateCycles=1

copyGraph!(fg_, fg, ls(fg, solvable=1), lsf(fg, solvable=1))

## load one of the PointCloud sets

de,db = getData(fg, :x0, :RADAR_SWEEP)
sweep = deserialize(PipeBuffer(db)) # BSON.@load
XY = map(pt->pt.data[1:2], sweep.points)
XY_ = filter(pt->10 < norm(pt), XY)
r0 = manikde!(getManifold(Point2), XY_, bw=[2;2.])

##

sap = ScatterAlignPose2(;hgd1=r0, hgd2=r10)

## show factor alignment plots

snt = overlayScatterMutate(sap; sample_count=300, bw=0.0001, user_coords=[-50.;0;0]);
plotScatterAlign(snt;title="\n#smpl=$(300)")

## visualize the radar data

imgs = map(x->Gray{N0f8}.(x), fetchDataImage.(fg, sortDFG(ls(fg)), :RADAR_IMG));
writevideo("/tmp/caesar/philos/radar.ogv", imgs; fps=3, player="totem")




## load one of the PointCloud sets

de,db = getData(fg, :x0, :RADAR_SWEEP)
sweep = deserialize(PipeBuffer(db)) # BSON.@load
XY = map(pt->pt.data[1:2], sweep.points)
XY_ = filter(pt->10 < norm(pt), XY)
r0 = manikde!(getManifold(Point2), XY_, bw=[2;2.])

##

sap = ScatterAlignPose2(;hgd1=r0, hgd2=r10)

## show factor alignment plots

snt = overlayScatterMutate(sap; sample_count=300, bw=0.0001, user_coords=[-50.;0;0]);
plotScatterAlign(snt;title="\n#smpl=$(300)")








# ## ===

# using Gadfly
# Gadfly.set_default_plot_size(40cm,20cm)


# ##

# lb = :x0
# de,db = getData(fg, lb, :RADAR_SWEEP)
# pointcloud = deserialize(PipeBuffer(db)) 

# X = (c->c.x).(pointcloud.points)
# Y = (c->c.y).(pointcloud.points)

# ##

# PL = []
# push!(PL, Gadfly.layer(x=X, y=Y, Geom.point))

# Gadfly.plot(PL...)

# ## Converting PCLPointCloud2 again


# PL = []

# for lb in [:x0;] #:x1;:x2;:x3;:x4;:x5]
#     de,db = getData(fg, lb, :RADAR_PC2s)
#     queueScans = deserialize(PipeBuffer(db)) 
#     for pc2 in queueScans[1:end]
#         pc_ = Caesar._PCL.PointCloud(pc2)
#         X = (c->c.x).(pc_.points)
#         Y = (c->c.y).(pc_.points)
#         push!(PL, Gadfly.layer(x=X, y=Y, Geom.point))
#     end
# end

# Gadfly.plot(PL...)

# ##


