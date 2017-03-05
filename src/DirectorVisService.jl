
# include("VisualizationUtilities.jl")  # @pyimport getimages as gi

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






function drawdbsession(vis, cloudGraph, attrdict, dcamjl, DRAWDEPTH, dbcoll, poseswithdepth)

  sesssym = Symbol(attrdict["session"])

  fg = Caesar.initfg(sessionname=attrdict["session"], cloudgraph=cloudGraph)
  IDs = getPoseExVertexNeoIDs(fg.cg.neo4j.connection, sessionname=attrdict["session"], reqbackendset=false);
  println("get local copy of graph")
  if fullLocalGraphCopy!(fg, reqbackendset=false)
    visualizeallposes!(vis, fg)

    xx,ll = ls(fg)
  	# LD = Array{Array{Float64,1},1}()
  	# C = Vector{AbstractString}()
  	for x in xx
  		val = getVal(fg,x)
  		len = size(val,2)
  		# [push!(C,"g") for i in 1:len];
      # assuming 2D here
      pointcloud = PointCloud([vec([val[1:2,i];0]) for i in 1:len])
      setgeometry!(vis[sesssym][:posepts][x], pointcloud)

      mongk, cvid = getmongokeys(fg, x, IDs)
  		if DRAWDEPTH && haskey(mongk, "depthframe_image") && !haskey(poseswithdepth, x)
        poseswithdepth[x]=1
        mongo_keydepth = bson.ObjectId(mongk["depthframe_image"])
        img, ims = gi.fastdepthimg(dbcoll, mongo_keydepth)
        X = reconstruct(dcamjl, Array{Float64,2}(img))
        r,c,h = size(X)
        Xd = X[1:10:r,1:10:c,:]
        rd,cd,hd = size(Xd)
        mask = Xd[:,:,:] .> 4.5
        Xd[mask] = Inf
        rgb = nothing
        seg = nothing
        if haskey(mongk, "keyframe_rgb")
          mongo_key = bson.ObjectId(mongk["keyframe_rgb"])
          rgb, ims = gi.fastrgbimg(dbcoll, mongo_key)
          # @show size(rgb)
        end
        if haskey(mongk, "keyframe_segnet")
          mongo_key = bson.ObjectId(mongk["keyframe_segnet"])
          seg, ims = gi.fastrgbimg(dbcoll, mongo_key)
        end
        if rgb != nothing
          rgbss = rgb[1:10:r,1:10:c,:]
          # bedu.publish_cloud("depth", Xd, c=rgbss, frame_id="MAPcams",element_id=j, flip_rb=true, reset=false)
          pts = Vector{Vector{Float64}}()
          for i in 1:rd, j in 1:cd
            if !isnan(Xd[i,j,1]) && Xd[i,j,3] != Inf
              push!(pts, vec(Xd[i,j,:]) )
            end
          end
          pointcloud = PointCloud(pts)
          println("drawing point cloud for $(string(x)), size $(size(pts))")
          setgeometry!(vis[sesssym][:poses][x][:pc], pointcloud)
          sleep(0.2)
        end
        # if seg != nothing
        #   segss = seg[1:3:r,1:3:c,:]
        #   bedu.publish_cloud("segnet", Xd, c=segss, frame_id="MAPcams",element_id=j, flip_rb=true, reset=false)
        # end
      end
    end


  end

end



function drawdbdirector()
  # Uncomment out for command line operation
  cloudGraph, attrdict = standardcloudgraphsetup(drawdepth=true)
  session = attrdict["session"]
  DRAWDEPTH = attrdict["draw depth"]=="y" # not going to support just yet

  # also connect to mongo separately
  client = pymongo.MongoClient(attrdict["mongo addr"])
  db = client[:CloudGraphs]
  collection = "bindata"

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
    drawdbsession(vis, cloudGraph, attrdict, dcamjl, DRAWDEPTH, db[collection], poseswithdepth)
    println(".")
  end

  println("Finishing askdrawdirectordb.")
  nothing
end
