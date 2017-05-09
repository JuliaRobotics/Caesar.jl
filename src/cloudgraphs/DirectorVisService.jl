
meshgrid(v::AbstractVector) = meshgrid(v, v)

function meshgrid{T}(vx::AbstractVector{T}, vy::AbstractVector{T})
    m, n = length(vy), length(vx)
    vx = reshape(vx, 1, n)
    vy = reshape(vy, m, 1)
    (repmat(vx, m, 1), repmat(vy, 1, n))
end

type DepthCamera
  K::Array{Float64,2}
  shape::Tuple{Int, Int}
  skip::Int
  D::Vector{Float64}
  xs::Array{Float64}
  ys::Array{Float64}
  DepthCamera(K::Array{Float64,2};
      shape::Tuple{Int,Int}=(480,640),
      skip::Int=1,
      D::Vector{Float64}=zeros(5) ) = new( K, shape, skip, D, Array{Float64,2}(), Array{Float64,2}() )
end

# Construct mesh for quick reconstruction
function buildmesh!(dc::DepthCamera)
  H, W = dc.shape
  xs,ys = collect(1:W), collect(1:H)
  fxinv = 1.0 / dc.K[1,1];
  fyinv = 1.0 / dc.K[2,2];

  xs = (xs-dc.K[1,3]) * fxinv
  xs = xs[1:dc.skip:end]
  ys = (ys-dc.K[2,3]) * fyinv
  ys = ys[1:dc.skip:end]

  dc.xs, dc.ys = meshgrid(xs, ys);
  nothing
end


function reconstruct(dc::DepthCamera, depth::Array{Float64})
  s = dc.skip
  depth_sampled = depth[1:s:end,1:s:end]
  # assert(depth_sampled.shape == self.xs.shape)
  r,c = size(dc.xs)

  ret = Array{Float64,3}(r,c,3)
  ret[:,:,1] = dc.xs .* depth_sampled
  ret[:,:,2] = dc.ys .* depth_sampled
  ret[:,:,3] = depth_sampled
  return ret
end

type SubmapColorCheat
  colors
  SubmapColorCheat(;colors::Vector=
    [ RGB(0.651, 0.808, 0.890),
      RGB(0.122, 0.471, 0.706),
      RGB(0.698, 0.875, 0.541),
      RGB(0.200, 0.627, 0.172),
      RGB(0.984, 0.604, 0.600),
      RGB(0.890, 0.102, 0.110),
      RGB(0.992, 0.749, 0.043)]
  ) = new(colors)
end

function submapcolor(idx::Int, len::Int;
        submapcolors=SubmapColorCheat() )
  #
  n = idx%length(submapcolors.colors)+1
  smc = submapcolors.colors[n]
  return [smc for g in 1:len]
end

# function prepcolordepthcloud!{T <: ColorTypes.Colorant}( X::Array;
#       rgb::Array{T, 2}=Array{Colorant,2}(),
#       skip::Int=4,
#       maxrange::Float64=4.5 )
function prepcolordepthcloud!( cvid::Int,
      X::Array;
      rgb::Array=Array{Colorant,2}(),
      skip::Int=4,
      maxrange::Float64=4.5 )
  #
  pointcloud = nothing
  pccols = nothing
  havecolor = size(rgb,1) > 0
  if typeof(X) == Array{Float64,3}
    r,c,h = size(X)
    Xd = X[1:skip:r,1:skip:c,:]
    rd,cd,hd = size(Xd)
    mask = Xd[:,:,:] .> maxrange
    Xd[mask] = Inf

    rgbss = havecolor ? rgb[1:skip:r,1:skip:c] : nothing
    # rgbss = rgb[1:4:r,1:4:c,:]./255.0
    pts = Vector{Vector{Float64}}()
    pccols = Vector()
    for i in 1:rd, j in 1:cd
      if !isnan(Xd[i,j,1]) && Xd[i,j,3] != Inf
        push!(pts, vec(Xd[i,j,:]) )
        havecolor ? push!(pccols, rgbss[i,j] ) : nothing
        # push!(pccols, RGB(rgbss[i,j,3], rgbss[i,j,2], rgbss[i,j,1]) )
      end
    end
    pointcloud = PointCloud(pts)
  elseif typeof(X) == Array{Array{Float64,1},1}
    pointcloud = PointCloud(X)
    pccols = rgb # TODO: refactor
  elseif size(X,1)==0
    return nothing
  else
    error("dont know how to deal with data type=$(typeof(X)),size=$(size(X))")
  end
  if havecolor
    pointcloud.channels[:rgb] = pccols
  else
    #submap colors
    smc = submapcolor(cvid, length(X))
    pointcloud.channels[:rgb] = smc
  end
  return pointcloud
end

function getPointCloudFromKinect(data, dcamjl, imshape)
  ri,ci = imshape[1], imshape[2] # TODO -- hack should be removed since depth is array and should have rows and columns stored in Mongo
  arr = bin2arr(data, dtype=Float32) # should also store dtype for arr in Mongo
  img = reshape(arr, ci, ri)'
  reconstruct(dcamjl, Array{Float64,2}(img))
end

function getPointCloudFromBSON(data)
  buf = IOBuffer(data)
  st = takebuf_string(buf)
  bb = BSONObject(st)
  return map(x -> convert(Array, x), bb["pointcloud"])
end

function cachepointclouds!(cache::Dict, cv::CloudVertex, ke::AbstractString, param::Dict)
  if !haskey(cache, ke)
    data = getBigDataElement(cv,ke)
    if typeof(data) == Void
      # warn("unable to load $(ke) from Mongo, gives data type Void")
      return nothing
    end
    data = data.data
    if ke == "depthframe_image"
      cache[ke] = getPointCloudFromKinect(data, param["dcamjl"], param["imshape"])
      # ri,ci = param["imshape"][1], param["imshape"][2] # TODO -- hack should be removed since depth is array and should have rows and columns stored in Mongo
      # # arrdata = data.data
      # arr = bin2arr(data, dtype=Float32) # should also store dtype for arr in Mongo
      # img = reshape(arr, ci, ri)'
      # X = reconstruct(param["dcamjl"], Array{Float64,2}(img))
      # cache[ke] = X
    elseif ke == "BSONpointcloud"
      # deserialize BSON-encoded pointcloud
      cache[ke] = getPointCloudFromBSON(data)
      # buf = IOBuffer(data)
      # st = takebuf_string(buf)
      # bb = BSONObject(st)
      # ptarr = map(x -> convert(Array, x), bb["pointcloud"])
      # cache[ke] = ptarr
    end
  end
  nothing
end

function retrievePointcloudColorInfo!(cv::CloudVertex, va::AbstractString)
  rgb = Array{Colorant,2}()
  if !hasBigDataElement(cv, va)
    warn("could not find color map in mongo, $(va)")
    return rgb
  end
  data = getBigDataElement(cv, va).data
  if va == "keyframe_rgb" || va == "keyframe_segnet"
    rgb = ImageMagick.readblob(data);
  elseif va == "BSONcolors"
    buffer = IOBuffer(data)
    str = takebuf_string(buffer)
    bb = BSONObject(str)
    # TODO -- maybe better to do: map(f, x), where f(x),
    # see http://docs.julialang.org/en/stable/manual/style-guide/#do-no-write-x-f-x
    carr = map(x -> convert(Array{UInt8}, x), bb["colors"])
    # typeof(rgb) = Array{Array{Colorant,1},1}
    rgb = carr
  end

  return rgb
end

function drawpointcloud!(vis::DrakeVisualizer.Visualizer,
        poseswithdepth::Dict,
        vsym::Symbol,
        pointcloud,
        va,
        param::Dict,
        sesssym::Symbol;
        # imshape=(480,640),
        wTb::CoordinateTransformations.AbstractAffineMap=
              Translation(0,0,0.0) ∘ LinearMap(
              CoordinateTransformations.Quat(1.0, 0, 0, 0))   )
        # bTc::CoordinateTransformations.AbstractAffineMap=
        #       Translation(0,0,0.6) ∘ LinearMap(
        #       CoordinateTransformations.Quat(0.5, -0.5, 0.5, -0.5))  )
  #

  pcsym = Symbol(string("pc_", va != "none" ? va : "ID"))
  setgeometry!(vis[sesssym][pcsym][vsym][:pose], Triad())
  settransform!(vis[sesssym][pcsym][vsym][:pose], wTb) # also updated as parallel track
  setgeometry!(vis[sesssym][pcsym][vsym][:pose][:cam], Triad())
  settransform!(vis[sesssym][pcsym][vsym][:pose][:cam], param["bTc"] )
  setgeometry!(vis[sesssym][pcsym][vsym][:pose][:cam][:pc], pointcloud )

  # these poses need to be update if the point cloud is to be moved
  if !haskey(poseswithdepth,vsym)
    thetype = typeof(vis[sesssym][pcsym][vsym][:pose])
    poseswithdepth[vsym] = Vector{ thetype }()
  end
  push!(poseswithdepth[vsym], vis[sesssym][pcsym][vsym][:pose])

  nothing
end


function fetchdrawdepthcloudbycvid!(vis::DrakeVisualizer.Visualizer,
      cloudGraph::CloudGraphs.CloudGraph,
      cvid::Int,
      vsym::Symbol,
      poseswithdepth::Dict,
      param::Dict,
      sesssym::Symbol;
      depthcolormaps=Dict(),
      # imshape=(480,640),
      wTb::CoordinateTransformations.AbstractAffineMap=
            Translation(0,0,0.0) ∘ LinearMap(
            CoordinateTransformations.Quat(1.0, 0, 0, 0))   )
      # bTc::CoordinateTransformations.AbstractAffineMap=
      #       Translation(0,0,0.6) ∘ LinearMap(
      #       CoordinateTransformations.Quat(0.5, -0.5, 0.5, -0.5))  )
  #

  if !haskey(poseswithdepth, vsym)
    cache = Dict()

    # fetch copy of big data from CloudGraphs
    cv = CloudGraphs.get_vertex(cloudGraph, cvid, true )

    # depthcolormaps = could be one or more or these options
    # 0, 1 or 2+ color maps per pointcloud
    #  ("none" => "depthframe_image", "none" => "pointcloud")
    # or
    #  ("keyframe_rgb" => "depthframe_image",
    #  "keyframe_segnet" => "depthframe_image")
    # or
    #  ("colors" => "pointcloud")
    for (va, ke) in depthcolormaps
      # prep the detph pointcloud
      cachepointclouds!(cache, cv, ke, param)

      if haskey(cache, ke)
        rgb = Array{Colorant,2}()
        if va[1:4] != "none"
          rgb = retrievePointcloudColorInfo!(cv, va)
        end

        # add color information to the pointcloud
        pointcloud = prepcolordepthcloud!( cvid, cache[ke], rgb=rgb )

        # publish the pointcloud data to Director viewer
        if pointcloud != nothing
          drawpointcloud!(vis, poseswithdepth, vsym, pointcloud, va, param, sesssym, wTb=wTb)
        end
      end
    end
  end
  nothing
end

"""
    updateparallelposes!(vis, poseswithdepth, wTb=::CoordinateTransformations.AbstractAffineMap)

Update all triads listed in poseswithdepth[Symbol(vert.label)] with wTb. Prevents cycles in
remote tree viewer of DrakeVisualizer.
"""
function updateparallelposes!(vis::DrakeVisualizer.Visualizer,
      vert::Graphs.ExVertex,
      poseswithdepth::Dict;
      wTb::CoordinateTransformations.AbstractAffineMap=
            Translation(0,0,0.0) ∘ LinearMap(
            CoordinateTransformations.Quat(1.0, 0, 0, 0))    )
  #

  if haskey(poseswithdepth, Symbol(vert.label))
    for cp in poseswithdepth[Symbol(vert.label)]
      settransform!(cp, wTb)
    end
  end
  nothing
end

function fetchdrawposebycvid!(vis::DrakeVisualizer.Visualizer,
      cloudGraph::CloudGraphs.CloudGraph,
      cvid::Int,
      poseswithdepth::Dict,
      param::Dict;
      session::AbstractString="",
      depthcolormaps=Dict()  )
      # imshape=(480,640),
      # bTc::CoordinateTransformations.AbstractAffineMap=
      #       Translation(0,0,0.6) ∘ LinearMap(
      #       CoordinateTransformations.Quat(0.5, -0.5, 0.5, -0.5))  )
  #

  # skip big data elements at first
  cv = CloudGraphs.get_vertex(cloudGraph, cvid, false )
  vert = cloudVertex2ExVertex(cv)

  # extract and draw new poses
  wTb = drawpose!(vis, vert, session=session )
  # also draw pose points from variable marginal belief approximation KDE
  drawposepoints!(vis, vert, session=session )

  # also update any parallel transform paths, previous and new
  updateparallelposes!(vis, vert, poseswithdepth, wTb=wTb)

  # check if we can draw depth pointclouds, and add new ones to parallel transform paths
  fetchdrawdepthcloudbycvid!(vis,
        cloudGraph,
        cvid,
        Symbol(vert.label),
        poseswithdepth,
        param,
        Symbol(session),
        depthcolormaps=depthcolormaps,
        # imshape=imshape,
        wTb=wTb  )

  sleep(0.005)
  nothing
end

# dbcoll,
function drawdbsession(vis::DrakeVisualizer.Visualizer,
      cloudGraph::CloudGraphs.CloudGraph,
      addrdict,
      param::Dict,
      poseswithdepth::Dict )
      # bTc::CoordinateTransformations.AbstractAffineMap=
      #       Translation(0,0,0.6) ∘ LinearMap(
      #       CoordinateTransformations.Quat(0.5, -0.5, 0.5, -0.5) )    )
  #

  @show session = addrdict["session"]
  sesssym = Symbol(session)
  DRAWDEPTH = addrdict["draw depth"]=="y" # not going to support just yet
  DRAWEDGES = addrdict["draw edges"]=="y" # not going to support just yet


  # fg = Caesar.initfg(sessionname=addrdict["session"], cloudgraph=cloudGraph)
  println("Fetching pose IDs to be drawn...")
  IDs = getExVertexNeoIDs(cloudGraph.neo4j.connection, label="POSE", session=session, reqbackendset=false);
  landmIDs = getExVertexNeoIDs(cloudGraph.neo4j.connection, label="LANDMARK", session=session, reqbackendset=false);

  @showprogress 1 "Drawing LANDMARK IDs..." for (vid,cvid) in landmIDs
    cv = nothing
    # skip big data elements
    cv = CloudGraphs.get_vertex(cloudGraph, cvid, false )
    vert = cloudVertex2ExVertex(cv)
    x = Symbol(vert.label)

    # vert = getVert(fg, x, api=localapi)
    drawpoint!(vis, vert, session=session)
    # drawposepoints!(vis, vert, session=session )
  end

  depthcolormaps = !DRAWDEPTH  ? Dict() : Dict("keyframe_rgb" => "depthframe_image", "keyframe_segnet" => "depthframe_image", "BSONcolors" => "BSONpointcloud", "none"=>"BSONpointcloud")

  @showprogress 1 "Drawing POSE IDs..." for (vid,cvid) in IDs
    fetchdrawposebycvid!(vis,
          cloudGraph,
          cvid,
          poseswithdepth,
          param,
          session=session,
          depthcolormaps=depthcolormaps  )
  end

  if DRAWEDGES
    println("Going to draw edges...")
    drawAllBinaryFactorEdges!(vis, cloudGraph, session)
  end
  nothing
end

function robotsetup(cg::CloudGraph, session::AbstractString)
  resp = fetchrobotdatafirstpose(cg, session)

  if haskey(resp, "CAMK")
    # CAMK = [[ 570.34222412; 0.0; 319.5]';
    #  [   0.0; 570.34222412; 239.5]';
    #  [   0.0; 0.0; 1.0]'];
    dcamjl = DepthCamera(resp["CAMK"])
    buildmesh!(dcamjl)
    resp["dcamjl"] = dcamjl
  end

  if haskey(resp, "bTc")
    bTc = Translation(0.0,0,0) ∘ LinearMap( CoordinateTransformations.Quat(1.0,0,0,0) )
    if resp["bTc_format"] == "xyzqwqxqyqz"
      bTc = Translation(resp["bTc"][1:3]...) ∘ LinearMap( CoordinateTransformations.Quat(resp["bTc"][4:7]...) )
    else
      warn("Unknown bTc_format, assuming identity for bTc")
    end
    resp["bTc"] = bTc
  end
  resp
end

function drawdbdirector(;addrdict::VoidUnion{Dict{AbstractString, AbstractString}}=nothing)
  # Uncomment out for command line operation
  cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict,drawdepth=true, drawedges=true)
  session = addrdict["session"]

  poseswithdepth = Dict()
  # poseswithdepth[:x1] = 0 # skip this pose -- there is no big data before ICRA

  vis = startdefaultvisualization()
  sleep(1.0)

  param = Dict()
  try
    param = robotsetup(cloudGraph, session)
  catch
    warn("No robot parameters found for the session, continuing with basic visualization only")
  end

  drawloop = Bool[true]
  println("Starting draw loop...")
  while drawloop[1]
    drawdbsession(vis, cloudGraph, addrdict, param, poseswithdepth) #,  db[collection]
    println(".")
    sleep(0.5)
  end

  println("Finishing askdrawdirectordb.")
  nothing
end
