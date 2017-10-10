# some drawing utils

using DrakeVisualizer, CoordinateTransformations, GeometryTypes, Rotations, TransformUtils, ColorTypes

# create a new Director window with home axis
function startdefaultvisualization(;newwindow=true,draworigin=true)
  DrakeVisualizer.any_open_windows() || DrakeVisualizer.new_window(); #DrakeVisualizer.new_window()
  viz = DrakeVisualizer.Visualizer()
  if draworigin
    setgeometry!(viz[:origin], Triad())
    settransform!(viz[:origin], Translation(0, 0, 0.0) ∘ LinearMap(Rotations.Quat(1.0,0,0,0)))
  end

  # realtime, rttfs = Dict{Symbol, Any}(), Dict{Symbol, AbstractAffineMap}()
  # dc = VisualizationContainer(Dict{Symbol, Visualizer}(), triads, trposes, meshes, realtime, rttfs)
  # visualizetriads!(dc)
  return viz
end


function visualizeDensityMesh!(vc::DrakeVisualizer.Visualizer, fgl::FactorGraph, lbl::Symbol; levels=3, meshid::Int=2)

  pl1 = marginal(getVertKDE(fgl,lbl),[1;2;3])

  gg = (x, a=0.0) -> evaluateDualTree(pl1, [x[1] x[2] x[3]]')[1]-a  #([x[1];x[2];x[3]]')'

  x = getKDEMax(pl1)
  maxval = gg(x)

  vv = getKDERange(pl1)
  lower_bound = Vec(vec(vv[:,1])...)
  upper_bound = Vec(vec(vv[:,2])...)

  levels = linspace(0.0,maxval,levels+2)

  # MD = []
  for val in levels[2:(end-1)]
    meshdata = GeometryData(contour_mesh(x -> gg(x,val), lower_bound, upper_bound))
    meshdata.color = RGBA( val/(1.5*maxval),0.0,1.0,val/(1.5*maxval))
    # push!(MD, meshdata)
    setgeometry!(vc[:meshes][lbl][Symbol("lev$(val)")], meshdata)
  end
  # mptr = Any(MD)
  # vc.meshes[lbl] = mptr
  # Visualizer(mptr, meshid) # meshdata
  nothing
end

type ArcPointsRangeSolve <: Function
  x1::Vector{Float64}
  x2::Vector{Float64}
  x3::Vector{Float64}
  r::Float64
  center::Vector{Float64}
  angle::Float64
  axis::Vector{Float64}
  ArcPointsRangeSolve(x1::Vector{Float64}, x2::Vector{Float64}, r::Float64) = new(x1,x2,zeros(0),r, zeros(2), 0.0, zeros(3))
  ArcPointsRangeSolve(x1::Vector{Float64}, x2::Vector{Float64}, x3::Vector{Float64}, r::Float64) = new(x1,x2,x3,r, zeros(3), 0.0, zeros(3))
end

function (as::ArcPointsRangeSolve)(x::Vector{Float64}, res::Vector{Float64})
  res[1] = norm(x-as.x1)^2 - as.r^2
  res[2] = norm(x-as.x2)^2 - as.r^2
  if length(res) == 3
    res[3] = norm(x-as.x3)^2 - as.r^2
  end
  nothing
end

function findaxiscenter!(as::ArcPointsRangeSolve)
  d = length(as.center)
  x0 = 0.5*(as.x1+as.x2)
  r = nlsolve(as, x0)
  as.center = r.zero
  vA, vB, vC = as.x1-as.center, as.x2-as.center, as.x3-as.center
  l1, l2 = norm(as.x1-as.x2), norm(as.x2-as.x3)
  halfl0 = 0.5*norm(as.x1-as.x3)
  axt = l1 < l2 ? Base.cross(vA,vB) : Base.cross(vB,vC)
  as.axis[1:3] = axt / norm(axt)
  ta = Base.cross(vA,vC)
  ta ./= norm(ta)
  alph = acos(halfl0/as.r)
  if norm(ta-as.axis) < 1e-4
    #accute
    as.angle = pi - 2*alph
  else
    # oblique
    as.angle = pi + 2*alph
  end
  r.f_converged
end

# as = ArcPointsRangeSolve([-1.0;0],[2.0;0],1.5)
# nlsolve(as, [1.0;1.0])


# find and set initial transform to project model in the world frame to the
# desired stating point and orientation
function findinitaffinemap!(as::ArcPointsRangeSolve; initrot::Rotation=Rotations.Quaternion(1.0,0,0,0))
  # how to go from origin to center to x1 of arc
  cent = Translation(as.center)
  rho = Translation(as.r, 0,0)
  return
end




# DrakeVisualizer.new_window()
# vctest = testtriaddrawing()



function drawpose!(viz::DrakeVisualizer.Visualizer, sym::Symbol;
      tf::CoordinateTransformations.AbstractAffineMap=Translation(0.0,0,0)∘LinearMap(CoordinateTransformations.AngleAxis(0.0,0,0,1.0)),
      session::AbstractString="",
      collection::Symbol=:poses)
  #
  if session == ""
    setgeometry!(viz[collection][sym], Triad())
    settransform!(viz[collection][sym], tf)
  else
    sesssym=Symbol(session)
    setgeometry!(viz[sesssym][collection][sym], Triad())
    settransform!(viz[sesssym][collection][sym], tf)
  end
  nothing
end

function drawpoint!(viz::DrakeVisualizer.Visualizer,
      sym::Symbol;
      tf=Translation(0.0,0,0),
      session::AbstractString="",
      scale=0.05,
      color=RGBA(0., 1, 0, 0.5),
      collection::Symbol=:landmarks  )
  #

  sphere = HyperSphere(Point(0., 0, 0), scale)
  csph = GeometryData(sphere, color)
  if session == ""
    setgeometry!(viz[collection][sym], csph)
    settransform!(viz[collection][sym], tf)
  else
    sesssym=Symbol(session)
    setgeometry!(viz[sesssym][collection][sym], csph)
    settransform!(viz[sesssym][collection][sym], tf)
  end
  nothing
end

function gettopoint(drawtype::Symbol=:max)
  topoint = +
  if drawtype == :max
    topoint = getKDEMax
  elseif drawtype == :mean
    topoint = getKDEMean
  elseif drawtype == :fit
    topoint = (x) -> getKDEfit(x).μ
  else
    error("Unknown draw type")
  end
  return topoint
end

function getdotwothree(sym::Symbol, X::Array{Float64,2})
  dims = size(X,1)
  dotwo = dims == 2 || (dims == 3 && string(sym)[1] == 'x')
  dothree = dims == 6 || (string(sym)[1] == 'l' && dims != 2)
  (dotwo && dothree) || (!dotwo && !dothree) ? error("Unknown dimension for drawing points in viewer, $((dotwo, dothree))") : nothing
  return dotwo, dothree
end

function drawpose!(vc::DrakeVisualizer.Visualizer,
      vert::Graphs.ExVertex,
      topoint::Function,
      dotwo::Bool, dothree::Bool;
      session::AbstractString="NA"  )
  #
  den = getVertKDE(vert)
  p = Symbol(vert.label)
  pointval = topoint(den)
  tf = nothing
  if dothree
    q = convert(TransformUtils.Quaternion, Euler(pointval[4:6]...))
    tf = Translation(pointval[1:3]...)∘LinearMap(Quat(q.s,q.v...))
    drawpose!(vc, p, tf=tf, session=session)
  elseif dotwo
    tf = Translation(pointval[1],pointval[2],0.0)∘LinearMap(Rotations.AngleAxis(pointval[3],0,0,1.0))
    drawpose!(vc, p, tf=tf, session=session)
  end
  return tf
end

function drawpose!(vc::DrakeVisualizer.Visualizer,
      vert::Graphs.ExVertex;
      session::AbstractString="NA",
      drawtype::Symbol=:max )
  #

  topoint = gettopoint(drawtype)
  X = getVal(vert)
  dotwo, dothree = getdotwothree(Symbol(vert.label), X)
  drawpose!(vc, vert, topoint, dotwo, dothree, session=session)
end

function drawpoint!(vc::DrakeVisualizer.Visualizer,
      vert::Graphs.ExVertex,
      topoint::Function,
      dotwo::Bool, dothree::Bool;
      session::AbstractString="NA"  )
  #
  den = getVertKDE(vert)
  p = Symbol(vert.label)
  pointval = topoint(den)
  if dothree
    q = convert(TransformUtils.Quaternion, Euler(pointval[4:6]...))
    drawpoint!(vc, p, tf=Translation(pointval[1:3]...), session=session)
  elseif dotwo
    drawpoint!(vc, p, tf=Translation(pointval[1],pointval[2],0.0), session=session)
  end
  nothing
end


function drawpoint!(vc::DrakeVisualizer.Visualizer,
        vert::Graphs.ExVertex;
        session::AbstractString="NA",
        drawtype::Symbol=:max )
  #
  topoint = gettopoint(drawtype)
  X = getVal(vert)
  dotwo, dothree = getdotwothree(Symbol(vert.label), X)
  drawpoint!(vc, vert, topoint, dotwo, dothree, session=session)
  nothing
end

function drawgt!(vc::DrakeVisualizer.Visualizer, sym::Symbol,
      gtval::Tuple{Symbol, Vector{Float64}};
      session::AbstractString="NA"  )
  #
  if gtval[1] == :XYZ
    drawpoint!(vc, sym, tf=Translation(gtval[2][1],gtval[2][2],gtval[2][3]),
          session=session,
          color=RGBA(1.0,0,0,0.5),
          collection=:gt_landm  )
  elseif gtval[1] == :XYZqWXYZ
    drawpose!(vc, sym,
          tf = Translation(gtval[2][1],gtval[2][2],gtval[2][3]) ∘
               LinearMap(CoordinateTransformations.Quat(gtval[2][4],gtval[2][5],gtval[2][6],gtval[2][7])),
          session=session,
          collection=:gt_poses  )
  else
    warn("unknown ground truth drawing type $(gtval[1])")
  end

  nothing
end

# TODO -- maybe we need RemoteFactorGraph type
function visualizeallposes!(vc::DrakeVisualizer.Visualizer,
    fgl::FactorGraph;
    drawlandms::Bool=true,
    drawtype::Symbol=:max,
    gt::Dict{Symbol, Tuple{Symbol,Vector{Float64}}}=Dict{Symbol, Tuple{Symbol,Vector{Float64}}}(),
    api::DataLayerAPI=localapi )
  #
  session = fgl.sessionname
  topoint = gettopoint(drawtype)

  dotwo = false
  dothree = false
  po,ll = ls(fgl)
  if length(po) > 0
    sym = po[1]
    X = getVal(fgl, sym, api=api )
    dotwo, dothree = getdotwothree(sym, X)
  end

  # TODO -- move calls higher in abstraction to be more similar to drawdbdirector()
  for p in po
    vert = getVert(fgl, p, api=api )
    drawpose!(vc, vert, topoint, dotwo, dothree, session=session)
    if haskey(gt, p)
      drawgt!(vc, p, gt[p], session=session)
    end
  end
  if drawlandms
    for l in ll
      den = getVertKDE(fgl, l, api=api)
      pointval = topoint(den)
      drawpoint!(vc, l, tf=Translation(pointval[1:3]...), session=session)
      if haskey(gt, l)
        drawgt!(vc, l, gt[l], session=session)
      end
    end
  end

  nothing
end

function colorwheel(n::Int)
  # RGB(1.0, 1.0, 0)
  convert(RGB, HSV((n*30)%360, 1.0,0.5))
end

function drawposepoints!(vis::DrakeVisualizer.Visualizer,
      vert::Graphs.ExVertex;
      session::AbstractString="NA"  )
  #
  vsym = Symbol(vert.label)
  X = getVal(vert)

  dotwo, dothree = getdotwothree(vsym, X)
  makefromX = (X::Array{Float64,2}, i::Int) -> X[1:3,i]
  if dotwo
    makefromX = (X::Array{Float64,2}, i::Int) -> Float64[X[1:2,i];0.0]
  end

  XX = Vector{Vector{Float64}}()
  for i in 1:size(X,2)
    push!(XX, makefromX(X,i))
  end
  pointcloud = PointCloud(XX)
  if string(vsym)[1] == 'l'
    pointcloud.channels[:rgb] = [RGB(1.0, 1.0, 0) for i in 1:length(XX)]
  elseif string(vsym)[1] == 'x'
    pointcloud.channels[:rgb] = [colorwheel(vert.index) for i in 1:length(XX)]
  end
  setgeometry!(vis[Symbol(session)][:posepts][vsym], pointcloud)
  nothing
end

function drawposepoints!(vis::DrakeVisualizer.Visualizer,
      fgl::FactorGraph,
      sym::Symbol;
      session::AbstractString="NA",
      api::DataLayerAPI=dlapi  )
  #
  vert = getVert(fgl, sym, api=api)
  drawposepoints!(vis, vert, session=session, api=localapi) # definitely use localapi
  nothing
end



function deletemeshes!(vc::DrakeVisualizer.Visualizer)
  delete!(vc[:meshes])
end



function drawLine!(vispath, from::Vector{Float64}, to::Vector{Float64}; scale=0.01,color=RGBA(0,1.0,0,0.5))
  vector = to-from
  len = norm(vector)
  buildline = Float64[len, 0, 0]

  v = norm(buildline-vector) > 1e-10 ? Base.cross(buildline, vector)  : [0,0,1.0]
  axis = v/norm(v)
  angle = acos(dot(vector, buildline)/(len^2) )
  rot = LinearMap( CoordinateTransformations.AngleAxis(angle, axis...) )

  mol = HyperRectangle(Vec(0.0,-scale,-scale), Vec(len,scale,scale))
  molbox = GeometryData(mol, color)

  setgeometry!(vispath, molbox)
  settransform!(vispath, Translation(from...) ∘ rot )
  nothing
end


"""
    drawLineBetweenPose3(fr::Graphs.ExVertex, to::Graphs.ExVertex; scale=, color=  )

Draw a line segment between to vertices.
"""
function drawLineBetween!(vis::DrakeVisualizer.Visualizer,
        session::AbstractString,
        fr::Graphs.ExVertex,
        to::Graphs.ExVertex;
        scale=0.01,
        name::Symbol=:edges,
        subname::Union{Void,Symbol}=nothing,
        color=RGBA(0,1.0,0,0.5)   )
  #
  dotwo, dothree = getdotwothree(Symbol(fr.label), getVal(fr))

  xipt = zeros(3); xjpt = zeros(3);
  if dothree
    xi = marginal(getVertKDE( fr ),[1;2;3] )
    xj = marginal(getVertKDE( to ),[1;2;3] )
    xipt[1:3] = getKDEMax(xi)
    xjpt[1:3] = getKDEMax(xj)
  elseif dotwo
    xi = marginal(getVertKDE( fr ),[1;2] )
    xj = marginal(getVertKDE( to ),[1;2] )
    xipt[1:2] = getKDEMax(xi)
    xjpt[1:2] = getKDEMax(xj)
  end

  lbl = Symbol(string(fr.label,to.label))
  place = vis[Symbol(session)][name][lbl]
  if subname != nothing
    place = vis[Symbol(session)][name][subname][lbl]
  end
  drawLine!(place, xipt, xjpt, color=color, scale=scale )
  nothing
end


"""
    drawLineBetween(fgl::FactorGraph, fr::Symbol, to::Symbol; scale, color, api  )

Draw a line segment between to nodes in the factor graph.
"""
function drawLineBetween!(vis::DrakeVisualizer.Visualizer,
        fgl::FactorGraph,
        fr::Symbol,
        to::Symbol;
        scale=0.01,
        name::Symbol=:edges,
        subname::Union{Void,Symbol}=nothing,
        color=RGBA(0,1.0,0,0.5),
        api::DataLayerAPI=dlapi  )
  #
  v1 = getVert(fgl, fr, api=api)
  v2 = getVert(fgl, to, api=api)

  drawLineBetween!(vis,fgl.sessionname, v1,v2,scale=scale,name=name,subname=subname,color=color   )
end

"""
    drawAllOdometryEdges!(fgl::FactorGraph, fr::Symbol, to::Symbol; scale, color, api  )

Assume odometry chain and draw edges between subsequent poses. Use keyword arguments to change colors, etc.
"""
function drawAllOdometryEdges!(vis::DrakeVisualizer.Visualizer,
      fgl::FactorGraph;
      scale=0.01,
      name::Symbol=:edges,
      color=RGBA(0,1.0,0,0.5),
      api::DataLayerAPI=dlapi  )
  #
  xx, ll = ls(fgl)

  for i in 1:(length(xx)-1)
    drawLineBetween!(vis, fgl, xx[i],xx[i+1], api=api , color=color, scale=scale, name=name )
  end

  nothing
end





function findAllBinaryFactors(fgl::FactorGraph; api::DataLayerAPI=dlapi)
  xx, ll = ls(fgl)

  slowly = Dict{Symbol, Tuple{Symbol, Symbol, Symbol}}()
  for x in xx
    facts = ls(fgl, x, api=localapi) # TODO -- known BUG on ls(api=dlapi)
    for fc in facts
      nodes = lsf(fgl, fc)
      if length(nodes) == 2
        # add to dictionary for later drawing
        if !haskey(slowly, fc)
          fv = getVert(fgl, fgl.fIDs[fc])
          slowly[fc] = (nodes[1], nodes[2], typeof(getfnctype(fv)).name.name)
        end
      end
    end
  end

  return slowly
end




function pointToColor(nm::Symbol)
  if nm == :PartialPose3XYYaw
    return RGBA(0.6,0.8,0.9,0.5)
  elseif nm == :Pose3Pose3NH
    return RGBA(1.0,1.0,0,0.5)
  else
    # println("pointToColor(..) -- warning, using default color for edges")
    return RGBA(0.0,1,0.0,0.5)
  end
end


function drawAllBinaryFactorEdges!(vis::DrakeVisualizer.Visualizer,
      fgl::FactorGraph;
      scale=0.01,
      api::DataLayerAPI=dlapi )
  #
  sloth = findAllBinaryFactors(fgl, api=api)

  for (teeth, toe) in sloth
    color = pointToColor(toe[3])
    drawLineBetween!(vis, fgl, toe[1], toe[2], subname=toe[3], scale=scale, color=color)
  end
  nothing
end





#
