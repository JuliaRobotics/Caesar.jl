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

  gg = (x, a=0.0) -> evaluateDualTree(pl1, ([x[1];x[2];x[3]]')')[1]-a

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
  axt = l1 < l2 ? cross(vA,vB) : cross(vB,vC)
  as.axis[1:3] = axt / norm(axt)
  ta = cross(vA,vC)
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
      wTrb=Translation(0.0,0,0),
      session::AbstractString="",
      scale=0.05,
      color=RGBA(0., 1, 0, 0.5),
      collection::Symbol=:landmarks  )
  #

  sphere = HyperSphere(Point(0., 0, 0), scale)
  csph = GeometryData(sphere, color)
  if session == ""
    setgeometry!(viz[collection][sym], csph)
    settransform!(viz[collection][sym], wTrb)
  else
    sesssym=Symbol(session)
    setgeometry!(viz[sesssym][collection][sym], csph)
    settransform!(viz[sesssym][collection][sym], wTrb)
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
  (dotwo && dothree) || (!dotwo && !dothree) ? error("Unknown dimension for drawing points in viewer") : nothing
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
  if dothree
    q = convert(TransformUtils.Quaternion, Euler(pointval[4:6]...))
    drawpose!(vc, p, tf=Translation(pointval[1:3]...)∘LinearMap(Quat(q.s,q.v...)), session=session)
  elseif dotwo
    drawpose!(vc, p, tf=Translation(pointval[1],pointval[2],0.0)∘LinearMap(Rotations.AngleAxis(pointval[3],0,0,1.0)), session=session)
  end
  nothing
end

function drawpose!(vc::DrakeVisualizer.Visualizer,
      vert::Graphs.ExVertex;
      session::AbstractString="NA",
      drawtype::Symbol=:max )
  #

  topoint = gettopoint(drawtype)
  X = getVal(vert)
  dotwo, dothree = getdotwothree(Symbol(vert.label), X)
  drawpose!(vc, vert, topoint, dotwo, dothree, session)
  nothing
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
  drawpoint!(vc, vert, topoint, dotwo, dothree, session)
  nothing
end

function drawgt!(vc::DrakeVisualizer.Visualizer, sym::Symbol,
      gtval::Tuple{Symbol, Vector{Float64}};
      session::AbstractString="NA"  )
  #
  if gtval[1] == :XYZ
    drawpoint!(vc, sym, wTrb=Translation(gtval[2][1],gtval[2][2],gtval[2][3]),
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
      drawpoint!(vc, l, wTrb=Translation(pointval[1:3]...), session=session)
      if haskey(gt, l)
        drawgt!(vc, l, gt[l], session=session)
      end
    end
  end

  nothing
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







#
