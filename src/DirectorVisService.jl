


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
  @show r,c = size(dc.xs)
  @show size(dc.ys)
  @show size(depth)

  ret = Array{Float64,3}(r,c,3)
  ret[:,:,1] = dc.xs .* depth_sampled
  ret[:,:,2] = dc.ys .* depth_sampled
  ret[:,:,3] = depth_sampled
  return ret
end






function drawdbsession(vis, cloudGraph, attrdict)


  fg = Caesar.initfg(sessionname=attrdict["session"], cloudgraph=cloudGraph)
  IDs = getPoseExVertexNeoIDs(fg.cg.neo4j.connection, sessionname=attrdict["session"], reqbackendset=false);
  println("get local copy of graph")
  if fullLocalGraphCopy!(fg, reqbackendset=false)
    visualizeallposes!(vis, fg)

    xx,ll = ls(fg)
  	LD = Array{Array{Float64,1},1}()
  	C = Vector{AbstractString}()
  	for x in xx
  		val = getVal(fg,x)
  		len = size(val,2)
  		# [push!(C,"g") for i in 1:len];
      # assuming 2D here
      pointcloud = PointCloud([vec([val[1:2,i];0]) for i in 1:len])
      setgeometry!(vis[:posepts][x], pointcloud)
  	end


  end

end


function askdrawdirectordb()
  # Uncomment out for command line operation
  cloudGraph, attrdict = standardcloudgraphsetup(drawdepth=true)
  session = attrdict["session"]
  # @show DRAWDEPTH = attrdict["draw depth"]=="y" # not going to support just yet

  vis = startdefaultvisualization()
  sleep(1.0)

  drawloop = Bool[true]
  while drawloop[1]
    drawdbsession(vis, cloudGraph, attrdict)
    println(".")
  end

  println("Finishing askdrawdirectordb.")
  nothing
end
