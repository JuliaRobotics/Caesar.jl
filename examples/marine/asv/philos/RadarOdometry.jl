
using Distributed
addprocs(10)

using Colors
using Caesar
@everywhere using Caesar

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
  writevideo("/tmp/caesar/philos/radar5.mp4", imgs; fps=5, player="totem")
end


## For this example, lets only add a few transform factors between poses, say every 10th pose

# set case specific parameters
getSolverParams(fg).inflateCycles=1
getSolverParams(fg).inflation=2.0

# switch of all variables
setSolvable!.(fg, ls(fg), 0)

# select the pose variables to include in the solve
slv = [Symbol("x",i) for i in 0:5:285]
# setSolvable!.(fg, slv, 1)

# add a PriorPose2
if 0==length(lsf(fg, tags=[:ORIGIN]))
  addFactor!(fg, [:x0;], PriorPose2(MvNormal([0;0;0.],diagm(0.01*[1;1;1.]))), tags=[:ORIGIN;], solvable=1)
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
  sap = ScatterAlignPose2(;cloud1=r1, cloud2=r2, bw=0.0001, sample_count=200)

  # add the new factor to the graph
  addFactor!(fg, [lb; lb_], sap, inflation=0.0, solvable=0, tags=[:RADAR_ODOMETRY], graphinit=false)
end


## make a copy of this subgraph for for dev engineering and debugging only

fg_ = initfg()
getSolverParams(fg).inflateCycles=1
getSolverParams(fg).inflation = 2.0
getSolverParams(fg).alwaysFreshMeasurements = false

copyGraph!(fg_, fg, slv, union((ls.(fg, slv))...))


## many batch solves of graph copy

setSolvable!(fg_, :x0, 1)
setSolvable!(fg_, :x0f1, 1)

tree = buildTreeReset!(fg_)

let tree = tree
  for (i,lb) in enumerate(slv)
    # latest pose to solve (factors were set at previous cycle)
    setSolvable!(fg_, lb, 1)
    if isodd(i)
      @info "solve for" lb
      tree = solveTree!(fg_, tree; storeOld=true);
      saveDFG("/tmp/caesar/philos/results_4/x0_5_$(lb)", fg_)
    end

    # set factors for next cycle
    setSolvable!.(fg_, ls(fg_, lb), 1)
  end
end


#