# addprocs(4)

using Caesar, RoME
using IncrementalInference
using TransformUtils
using Rotations, CoordinateTransformations

using PyCall
using PyLCM
using Distributions

using CloudGraphs

using KernelDensityEstimate

using JLD, HDF5

using Colors, FixedPointNumbers

using DrakeVisualizer

using LibBSON

@show lcmtpath = joinpath(dirname(@__FILE__),"lcmtypes")
run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/hauv_submap_points_t.lcm`)
println("Adding lcmtypes dir to Python path: $(lcmtpath)")
unshift!(PyVector(pyimport("sys")["path"]),lcmtpath)
@pyimport hauv

function lcmpointcloudmsg!(vc, slaml::SLAMWrapper,
                          msgdata  )

  msg = hauv.submap_points_t[:decode](msgdata)

  vert = getVert(slaml.fg, slaml.lastposesym, api=IncrementalInference.dlapi) # fetch from database

  position = Translation(msg[:pos]...)
  orientation = Rotations.Quat(msg[:orientation]...)

  Tf = position ∘ LinearMap(orientation)

  vsym = Symbol(vert.label)

  setgeometry!(vc[:submaps][vsym], Triad())
  settransform!(vc[:submaps][vsym], Tf)

  # 2d arrays of points and colors
  ptarr = [[pt[1], pt[2], pt[3]] for pt in msg[:points_local]]
  carr = [[UInt8(c.data[1]),UInt8(c.data[2]),UInt8(c.data[3])] for c in msg[:colors]]

  # push to mongo (using BSON as a quick fix)
  serialized_point_cloud = BSONObject(Dict("pointcloud" => ptarr))
  appendvertbigdata!(slaml.fg, vert, "pointcloud", string(serialized_point_cloud).data)
  # appendvertbigdata!(slaml.fg, vert, "colors", carr)

  cv = CloudGraphs.get_vertex(slaml.fg.cg, slaml.fg.cgIDs[slaml.fg.IDs[vsym]], true) # big data fetch
  data = Caesar.getBigDataElement(cv, "pointcloud")
  buffer = IOBuffer(data.data)
  str = takebuf_string(buffer)
  bb = BSONObject(str)
  ptarr = map(x -> convert(Array, x), bb["pointcloud"])
  pointcloud = PointCloud(ptarr)
  setgeometry!(vc[:submaps][vsym][:pc], pointcloud)

  # serialize point cloud data
  #=
  serialized_point_cloud = BSONObject(Dict("pointcloud" => ptarr))
  serialized_colors = BSONObject(Dict("colors" => carr))
  =#

  # begin: move code below to drawdb (point cloud support)
  # deserialize
  #=
  ptarr = serialized_point_cloud["pointcloud"]
  carr = serialized_colors["colors"]
  ptarr = map(x -> convert(Array, x), ptarr)
  carr = map(x -> convert(Array{UInt8}, x), carr)
  =#
  # push to DrakeVisualizer
  #=
  pointcloud = PointCloud(ptarr)
  pointcloud.channels[:rgb] = map(x -> RGB(reinterpret(N0f8, x)...), carr)
  setgeometry!(vc[:submaps][vsym][:pc], pointcloud)
  =#
  # end

  nothing
end
  # begin: move code below to drawdb (point cloud support)
  # deserialize
  #=
  ptarr = serialized_point_cloud["pointcloud"]
  carr = serialized_colors["colors"]
  ptarr = map(x -> convert(Array, x), ptarr)
  carr = map(x -> convert(Array{UInt8}, x), carr)
  =#
  # push to DrakeVisualizer
  #=
  pointcloud = PointCloud(ptarr)
  pointcloud.channels[:rgb] = map(x -> RGB(reinterpret(N0f8, x)...), carr)
  setgeometry!(vc[:submaps][vsym][:pc], pointcloud)
  =#
  # end

  nothing
end
  # begin: move code below to drawdb (point cloud support)
  # deserialize
  #=
  ptarr = serialized_point_cloud["pointcloud"]
  carr = serialized_colors["colors"]
  ptarr = map(x -> convert(Array, x), ptarr)
  carr = map(x -> convert(Array{UInt8}, x), carr)
  =#
  # push to DrakeVisualizer
  #=
  pointcloud = PointCloud(ptarr)
  pointcloud.channels[:rgb] = map(x -> RGB(reinterpret(N0f8, x)...), carr)
  setgeometry!(vc[:submaps][vsym][:pc], pointcloud)
  =#
  # end

  nothing
end


function setupSLAMinDB(;cloudGraph=nothing, addrdict=nothing)
  if cloudGraph != nothing
    return SLAMWrapper(Caesar.initfg(sessionname=addrdict["session"], cloudgraph=cloudGraph), nothing, 0)
  else
    return SLAMWrapper(RoME.initfg(), nothing, 0)
  end
end


vc = startdefaultvisualization()
# sleep(3.0)
rovt = loadmodel(:rov)
rovt(vc)

Nparticles = 100

include(joinpath(dirname(@__FILE__),"..","database","blandauthremote.jl"))
addrdict["session"] = "SESSHAUVDEV"


# prepare the factor graph with just one node
cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)

slam = setupSLAMinDB(cloudGraph=cloudGraph, addrdict=addrdict)
slam.fg = identitypose6fg(fg=slam.fg)

ls(slam.fg)

# writeGraphPdf(slam.fg)
# run(`evince fg.pdf`)
# Base.rm("fg.pdf")
tree = wipeBuildNewTree!(fg,drawpdf=true)
run(`evince bt.pdf`)

fieldnames(slam)
slam.lastposesym

function something(vc, slaml)
  # do the LCM stuff
  lc = LCM()

  pchdl = (channel, msg_data) -> lcmpointcloudmsg!(vc, slaml, msg_data )

  subscribe(lc, "SUBMAPS", pchdl)

  # run this thing for one message only....
  handle(lc)
end

something(vc, slam)


Juno.breakpoint(@__FILE__(), 52)




















#
p
