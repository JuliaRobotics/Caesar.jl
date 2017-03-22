
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


function prepcolordepthcloud!( X, rgb; skip::Int=4, maxrange=4.5 )
  #
  r,c,h = size(X)
  Xd = X[1:skip:r,1:skip:c,:]
  rd,cd,hd = size(Xd)
  mask = Xd[:,:,:] .> maxrange
  Xd[mask] = Inf

  rgbss = rgb[1:skip:r,1:skip:c]
  # rgbss = rgb[1:4:r,1:4:c,:]./255.0
  pts = Vector{Vector{Float64}}()
  pccols = Vector()
  for i in 1:rd, j in 1:cd
    if !isnan(Xd[i,j,1]) && Xd[i,j,3] != Inf
      push!(pts, vec(Xd[i,j,:]) )
      push!(pccols, rgbss[i,j] ) # RBG
      # push!(pccols, RGB(rgbss[i,j,3], rgbss[i,j,2], rgbss[i,j,1]) )
    end
  end
  pointcloud = PointCloud(pts)
  pointcloud.channels[:rgb] = pccols
  return pointcloud
end


# dbcoll,
function drawdbsession(vis,
      cloudGraph,
      addrdict,
      dcamjl,
      DRAWDEPTH,
      poseswithdepth;
      bTc::CoordinateTransformations.AbstractAffineMap=
            Translation(0,0,0.6) ∘ LinearMap(
            CoordinateTransformations.Quat(0.5, -0.5, 0.5, -0.5) )     )
  #

  @show session = addrdict["session"]
  sesssym = Symbol(session)

  # fg = Caesar.initfg(sessionname=addrdict["session"], cloudgraph=cloudGraph)
  @show IDs = getPoseExVertexNeoIDs(cloudGraph.neo4j.connection, sessionname=session, reqbackendset=false);

  @showprogress 1 "Drawing IDs..." for (vid,cvid) in IDs

    @show vid, cvid
    cv = CloudGraphs.get_vertex(cloudGraph, cvid)
    vert = cloudVertex2ExVertex(cv)
    x = Symbol(vert.label)

    mongk = Dict{AbstractString, Any}()
    if haskey(cv.properties, "mongo_keys")
      jsonstr = cv.properties["mongo_keys"]
      mongk =  JSON.parse(jsonstr)
    end

    # vert = getVert(fg, x, api=localapi)
    drawpose!(vis, vert, session=session)
    drawposepoints!(vis, vert, session=session )

    # mongk, cvid = getmongokeys(fg, x, IDs)

		if DRAWDEPTH && haskey(mongk, "depthframe_image") && !haskey(poseswithdepth, x)
      poseswithdepth[x]=1

      rgb = nothing
      seg = nothing
      if haskey(mongk, "keyframe_rgb")
        rgb = fetchmongorgbimg(cloudGraph, mongk["keyframe_rgb"])
      end
      if haskey(mongk, "keyframe_segnet")
        seg = fetchmongorgbimg(cloudGraph, mongk["keyframe_segnet"])
      end

      ri,ci = size(rgb)
      arr = fetchmongodepthimg(cloudGraph, mongk["depthframe_image"], dtype=Float32)
      img = reshape(arr, ci, ri)'

      X = reconstruct(dcamjl, Array{Float64,2}(img))

      if rgb != nothing
        pointcloud = prepcolordepthcloud!( X, rgb )
        setgeometry!(vis[sesssym][:poses][x][:cam], Triad())
        settransform!(vis[sesssym][:poses][x][:cam], bTc)
        setgeometry!(vis[sesssym][:poses][x][:cam][:pc], pointcloud )
      end
      # if seg != nothing
      #   segss = seg[1:3:r,1:3:c,:]
      #   bedu.publish_cloud("segnet", Xd, c=segss, frame_id="MAPcams",element_id=j, flip_rb=true, reset=false)
      # end
    end
    sleep(0.005)
  end

end



function drawdbdirector()
  # Uncomment out for command line operation
  cloudGraph, addrdict = standardcloudgraphsetup(drawdepth=true)
  session = addrdict["session"]
  DRAWDEPTH = addrdict["draw depth"]=="y" # not going to support just yet

  poseswithdepth = Dict()
  poseswithdepth[:x1] = 0 # skip this pose -- there is no big data before ICRA

  vis = startdefaultvisualization()
  sleep(1.0)

  CAMK = [[ 570.34222412; 0.0; 319.5]';
   [   0.0; 570.34222412; 239.5]';
   [   0.0; 0.0; 1.0]'];
  dcamjl = DepthCamera(CAMK)
  buildmesh!(dcamjl)

  drawloop = Bool[true]
  while drawloop[1]
    drawdbsession(vis, cloudGraph, addrdict, dcamjl, DRAWDEPTH, poseswithdepth) #,  db[collection]
    println(".")
  end

  println("Finishing askdrawdirectordb.")
  nothing
end
