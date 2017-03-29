
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


function prepcolordepthcloud!( X;
      rgb::VoidUnion{Array{Colorant, 2}}=nothing,
      skip::Int=4,
      maxrange=4.5 )
  #
  r,c,h = size(X)
  Xd = X[1:skip:r,1:skip:c,:]
  rd,cd,hd = size(Xd)
  mask = Xd[:,:,:] .> maxrange
  Xd[mask] = Inf

  havecolor = rgb == nothing

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
  if havecolor
    pointcloud.channels[:rgb] = pccols
  end
  return pointcloud
end


function fetchdrawdepthcloudbycvid!(vis::DrakeVisualizer.Visualizer,
      cloudGraph::CloudGraphs.CloudGraph,
      cvid::Int,
      poseswithdepth::Dict{Symbol, Int};
      depthcolormaps=Dict(),
      imshape=(480,640),
      wTb::CoordinateTransformations.AbstractAffineMap=
            Translation(0,0,0.0) ∘ LinearMap(
            CoordinateTransformations.Quat(1.0, 0, 0, 0)),
      bTc::CoordinateTransformations.AbstractAffineMap=
            Translation(0,0,0.6) ∘ LinearMap(
            CoordinateTransformations.Quat(0.5, -0.5, 0.5, -0.5))  )
  #

  vsym = Symbol(vert.label)

  if !haskey(poseswithdepth, vsym)
    cache = Dict()
    # fetch copy of big data from CloudGraphs
    cv = CloudGraphs.get_vertex(cloudGraph, cvid, true )
    # "depthframe_image" => "keyframe_rgb", "depthframe_image" => "keyframe_segnet"
    for (ke, va) in depthcolormaps
      # TODO -- should be removed since depth is array and should have rows and columns stored in Mongo
      # size(rgb)
      ri,ci = imshape[1], imshape[2]

      if !haskey(cache, ke)
        arrdata = getBigDataElement(cv,ke).data #"depthframe_image"
        arr = bin2arr(arrdata, dtype=Float32) # should also store dtype for arr in Mongo
        img = reshape(arr, ci, ri)'
        X = reconstruct(dcamjl, Array{Float64,2}(img))
        cache[ke] = X
      end

      rgb = nothing
      if hasBigDataElement(cv, va) # "keyframe_rgb"
        rgbbigd = getBigDataElement(cv, va).data
        rgb = ImageMagick.readblob(rgbbigd);
      end
      pointcloud = prepcolordepthcloud!( cache[ke], rgb=rgb )

      pcsym = Symbol(string("pc_",va))
      setgeometry!(vis[sesssym][pcsym][vsym][:pose], Triad())
      settransform!(vis[sesssym][pcsym][vsym][:pose], wTb) # also updated as parallel track
      setgeometry!(vis[sesssym][pcsym][vsym][:pose][:cam], Triad())
      settransform!(vis[sesssym][pcsym][vsym][:pose][:cam], bTc )
      setgeometry!(vis[sesssym][pcsym][vsym][:pose][:cam][:pc], pointcloud )

      # these poses need to be update if the point cloud is to be moved
      if !(length(poseswithdepth[x]) > 0)
        @show thetype = typeof(vis[sesssym][pcsym][vsym][:pose])
        poseswithdepth[vsym] = Vector{ thetype }()
      end
      push!(poseswithdepth[vsym], vis[sesssym][pcsym][vsym][:pose])
    end # for depthcolormaps
  end # !haskey(poseswithdepth, ke)
  nothing
end

"""
    updateparallelposes!(vis, poseswithdepth, wTb=::CoordinateTransformations.AbstractAffineMap)

Update all triads listed in poseswithdepth[Symbol(vert.label)] with wTb. Prevents cycles in
remote tree viewer of DrakeVisualizer.
"""
function updateparallelposes!(vis::DrakeVisualizer.Visualizer, vert, poseswithdepth;
      wTb::CoordinateTransformations.AbstractAffineMap=
            Translation(0,0,0.0) ∘ LinearMap(
            CoordinateTransformations.Quat(1.0, 0, 0, 0))    )
  #

  @show Symbol(vert.label), length(poseswithdepth[Symbol(vert.label)])
  for cp in poseswithdepth[Symbol(vert.label)]
    @show cp
    settransform!(cp, wTb)
  end
  nothing
end

function fetchdrawposebycvid!(vis::DrakeVisualizer.Visualizer,
      cloudGraph::CloudGraphs.CloudGraph,
      cvid::Int,
      poseswithdepth::Dict{Symbol, Int};
      session::AbstractString="",
      depthcolormaps=Dict(),
      imshape=(480,640),
      bTc::CoordinateTransformations.AbstractAffineMap=
            Translation(0,0,0.6) ∘ LinearMap(
            CoordinateTransformations.Quat(0.5, -0.5, 0.5, -0.5))  )
  #

  # skip big data elements at first
  cv = CloudGraphs.get_vertex(cloudGraph, cvid, false )
  vert = cloudVertex2ExVertex(cv)

  # extract and draw new poses
  wTb = drawpose!(vis, vert, session=session )
  # also draw pose points from variable marginal belief approximation KDE
  drawposepoints!(vis, vert, session=session )

  @show vert.label, typeof(poseswithdepth[Symbol(vert.label)]), poseswithdepth[Symbol(vert.label)]

  # also update any parallel transform paths, previous and new
  updateparallelposes!(vis, vert, poseswithdepth, wTb=wTb)

  println("Draw depth cloud")
  # check if we can draw depth pointclouds, and add new ones to parallel transform paths
  fetchdrawdepthcloudbycvid!(vis, cloudGraph, cvid, poseswithdepth,
        depthcolormaps=depthcolormaps,
        imshape=imshape,
        wTb=wTb )

  sleep(0.005)
  nothing
end

# dbcoll,
function drawdbsession(vis::DrakeVisualizer.Visualizer,
      cloudGraph::CloudGraphs.CloudGraph,
      addrdict,
      dcamjl,
      DRAWDEPTH,
      poseswithdepth::Dict{Symbol, Int};
      bTc::CoordinateTransformations.AbstractAffineMap=
            Translation(0,0,0.6) ∘ LinearMap(
            CoordinateTransformations.Quat(0.5, -0.5, 0.5, -0.5) )     )
  #

  @show session = addrdict["session"]
  sesssym = Symbol(session)

  # fg = Caesar.initfg(sessionname=addrdict["session"], cloudgraph=cloudGraph)
  println("Fetching pose IDs to be drawn...")
  IDs = getExVertexNeoIDs(cloudGraph.neo4j.connection, label="POSE", session=session, reqbackendset=false);
  landmIDs = getExVertexNeoIDs(cloudGraph.neo4j.connection, label="LANDMARK", session=session, reqbackendset=false);

  @showprogress 1 "Drawing LANDMARK IDs..." for (vid,cvid) in landmIDs
    cv = nothing
    @show vid, cvid
    # skip big data elements
    cv = CloudGraphs.get_vertex(cloudGraph, cvid, false )
    vert = cloudVertex2ExVertex(cv)
    x = Symbol(vert.label)

    # vert = getVert(fg, x, api=localapi)
    drawpoint!(vis, vert, session=session)
    # drawposepoints!(vis, vert, session=session )
  end

  depthcolormaps = Dict("depthframe_image" => "keyframe_rgb", "depthframe_image" => "keyframe_segnet")

  @showprogress 1 "Drawing POSE IDs..." for (vid,cvid) in IDs
    @show vid, cvid
    fetchdrawposebycvid!(vis, cloudGraph, cvid, poseswithdepth, session=session,
                  depthcolormaps=depthcolormaps  )
  end

end



function drawdbdirector()
  # Uncomment out for command line operation
  cloudGraph, addrdict = standardcloudgraphsetup(drawdepth=true)
  session = addrdict["session"]
  DRAWDEPTH = addrdict["draw depth"]=="y" # not going to support just yet

  poseswithdepth = Dict{Symbol, Int}()
  poseswithdepth[:x1] = 0 # skip this pose -- there is no big data before ICRA

  vis = startdefaultvisualization()
  sleep(1.0)

  CAMK = [[ 570.34222412; 0.0; 319.5]';
   [   0.0; 570.34222412; 239.5]';
   [   0.0; 0.0; 1.0]'];
  dcamjl = DepthCamera(CAMK)
  buildmesh!(dcamjl)

  drawloop = Bool[true]
  println("Starting draw loop...")
  while drawloop[1]
    drawdbsession(vis, cloudGraph, addrdict, dcamjl, DRAWDEPTH, poseswithdepth) #,  db[collection]
    println(".")
  end

  println("Finishing askdrawdirectordb.")
  nothing
end
