using Caesar, RoME
using TransformUtils, Rotations, CoordinateTransformations
using PyCall, PyLCM
using LibBSON
# using Colors, FixedPointNumbers #needed if pushing to DrakeVisualizer

# generate python bindings for lcmtypes and add them to PYTHONPATH
@show lcmtpath = joinpath(dirname(@__FILE__),"lcmtypes")
run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/hauv_submap_points_t.lcm`)
println("Adding lcmtypes dir to Python path: $(lcmtpath)")
unshift!(PyVector(pyimport("sys")["path"]),lcmtpath)
@pyimport hauv

function lcmpointcloudmsg!(vc, slaml::SLAMWrapper,
                          msgdata  )
  msg = hauv.submap_points_t[:decode](msgdata)

  vert = getVert(slaml.fg, slaml.lastposesym, api=IncrementalInference.dlapi) # fetch from database

  # 2d arrays of points and colors (from LCM data into arrays{arrays})
  ptarr = [[pt[1], pt[2], pt[3]] for pt in msg[:points_local]]
  carr = [[UInt8(c.data[1]),UInt8(c.data[2]),UInt8(c.data[3])] for c in msg[:colors]]

  # push to mongo (using BSON as a quick fix)
  # (for deserialization, see src/DirectorVisService.jl:cachepointclouds!)
  serialized_point_cloud = BSONObject(Dict("pointcloud" => ptarr))
  appendvertbigdata!(slaml.fg, vert, "BSONpointcloud", string(serialized_point_cloud).data)
  serialized_colors = BSONObject(Dict("colors" => carr))
  appendvertbigdata!(slaml.fg, vert, "BSONcolors", string(serialized_colors).data)

  # push to DrakeVisualizer
  #=
  position = Translation(msg[:pos]...)
  orientation = _Rot.QuatRotation(msg[:orientation]...)
  Tf = position ∘ LinearMap(orientation)
  vsym = Symbol(vert.label)
  setgeometry!(vc[:submaps][vsym], Triad())
  settransform!(vc[:submaps][vsym], Tf)

  pointcloud = PointCloud(ptarr)
  pointcloud.channels[:rgb] = map(x -> RGB(reinterpret(N0f8, x)...), carr)
  setgeometry!(vc[:submaps][vsym][:pc], pointcloud)
  =#

  nothing
end

# this function is on notice!
function setupSLAMinDB(;cloudGraph=nothing, addrdict=nothing)
  if cloudGraph != nothing
    return SLAMWrapper(initfg(sessionname=addrdict["session"], cloudgraph=cloudGraph), nothing, 0)
  else
    return SLAMWrapper(RoME.initfg(), nothing, 0)
  end
end


# prepare the factor graph with just one node
# (will prompt on stdin for db credentials)
cloudGraph, addrdict = standardcloudgraphsetup()

slam = setupSLAMinDB(cloudGraph=cloudGraph, addrdict=addrdict)
slam.fg = identitypose6fg(fg=slam.fg)


function listenone(slaml)
  lc = LCM()

  # lcm callback closure
  pchdl = (channel, msg_data) -> lcmpointcloudmsg!( slaml, msg_data )

  subscribe(lc, "SUBMAPS", pchdl)

  # run this thing for one message only....
  handle(lc)
end

listenone(slam)

# Juno.breakpoint(@__FILE__(), 52)
