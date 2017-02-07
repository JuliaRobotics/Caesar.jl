# some drawing utils

using DrakeVisualizer, CoordinateTransformations, GeometryTypes, Rotations, TransformUtils, ColorTypes


type VisualizationContainer{T}
  models::Dict{T, Visualizer}
  triads::Dict{T, Any}
  triadposes::Dict{T, AbstractAffineMap}
  meshes::Dict{T, Any}
  realtime::Dict{T, Any}
  rttfs::Dict{T, AbstractAffineMap}
  # pointpositions::Dict{T, Translation}
end


function Anyaxestriad(;width=0.02,length=1.0)
  Xbox = GeometryData(HyperRectangle(Vec(0.,-width,-width), Vec(length,width,width)))
  Xbox.color = RGBA(1., 0, 0, 0.5)
  Ybox = GeometryData(HyperRectangle(Vec(-width,0.0,-width), Vec(width,length,width)))
  Ybox.color = RGBA(0., 1, 0, 0.5)
  Zbox = GeometryData(HyperRectangle(Vec(-width,-width,0.0), Vec(width,width,length)))
  Zbox.color = RGBA(0., 0, 1, 0.5)
  Any([Xbox;Ybox;Zbox])
end

function settriadpose!{T}(triadposes::Dict{T, AbstractAffineMap}, id::T, wTb::Translation, wRb::Rotation)
  triadposes[id] = wTb ∘ LinearMap(wRb)
  nothing
end
settriadpose!{T}(triadposes::Dict{T, AbstractAffineMap}, id::T, wTb::Vector{Float64}, wQb::TransformUtils.Quaternion) =
    updatetriad!(triadposes, id, Translation(wTb...), Quat(wQb.s, wQb.v...))

function setpointpose!{T}(positions::Dict{T, AbstractAffineMap}, id::T, wTb::Translation)
  positions[id] = wTb
  nothing
end

function newtriad!{T}(triads::Dict{T,Any}, triadposes::Dict{T, AbstractAffineMap}, id::T;
      wRb::Rotation=Quat(1.,0,0,0),
      wTb::Translation=Translation(0.,0,0),
      width=0.02,
      length=1.0  )
  triads[id] = Anyaxestriad(width=width, length=length)
  settriadpose!(triadposes, id, wTb, wRb)
  nothing
end

function newtriad!{T}(dc::VisualizationContainer{T}, id::T;
      wRb::Rotation=Quat(1.,0,0,0),
      wTb::Translation=Translation(0.,0,0),
      width=0.02,
      length=1.0 )
  #
  newtriad!(dc.triads, dc.triadposes, id, wRb=wRb, wTb=wTb, width=width, length=length)
end


function newpoint!{T}(dc::VisualizationContainer{T}, id::T;
      wTb::Translation=Translation(0.,0,0),
      radius=0.02,
      color=RGBA(0., 0, 1, 0.5) )
  #
  pt = GeometryData(HyperSphere(Point(0.,0,0), radius))
  pt.color = color
  dc.triads[id] = Any([pt])
  setpointpose!(dc.triadposes, id, wTb)
  nothing
end

# function visualizetriads{T}(triads::Dict{T,Any}, triadposes::Dict{T, AbstractAffineMap})
#   triadsmodel = Visualizer(triads)
#   DrakeVisualizer.draw(triadsmodel, triadposes)
#   triadsmodel
# end
function visualizetriads!(vc::VisualizationContainer)  # = visualizetriads(dc.triads, dc.triadposes)
  vc.models[:dev] = Visualizer(vc.triads, 1)
  DrakeVisualizer.draw(vc.models[:dev], vc.triadposes)
  nothing
end

function testtriaddrawing()
  # triads, trposes = Dict{Int,Any}(), Dict{Int, AbstractAffineMap}()
  vc = VisualizationContainer(nothing,Dict{Int,Any}(), Dict{Int, AbstractAffineMap}(), Dict{Int, HomogenousMesh}(), Dict{Int, Any}(), Dict{Int, AbstractAffineMap}())
  newtriad!(vc, 0)
  newtriad!(vc, 1,wTb=Translation(2.,0,0),wRb=CoordinateTransformations.AngleAxis(pi/4,1.,0,0),length=0.5)
  newtriad!(vc, 2,wTb=Translation(4.,0,0),wRb=CoordinateTransformations.AngleAxis(pi/2,1.,0,0),length=0.5)

  newpoint!(vc, 100, wTb=Translation(1.,1,0))

  visualizetriads!(vc)
  vc
end


# create a new Director window with home axis
function startdefaultvisualization(;newwindow=true,draworigin=true)
  DrakeVisualizer.new_window()
  triads, trposes, meshes = Dict{Symbol, Any}(), Dict{Symbol, AbstractAffineMap}(), Dict{Symbol, Any}()
  draworigin ? newtriad!(triads, trposes, :origin) : nothing
  realtime, rttfs = Dict{Symbol, Any}(), Dict{Symbol, AbstractAffineMap}()
  #newtriad!(vc,p, wTb=Translation(maxval[1:3]...), wRb=Quat(q.s,q.v...), length=0.5)
  dc = VisualizationContainer(Dict{Symbol, Visualizer}(), triads, trposes, meshes, realtime, rttfs)
  visualizetriads!(dc)
  # model = visualizetriads(triads, trposes)
  return dc
end


function visualizeallposes!(vc::VisualizationContainer, fgl::FactorGraph; drawlandms::Bool=true,drawtype::Symbol=:max)
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

  po,ll = ls(fgl)
  for p in po
    # v = getVert(fgl, p)
    den = getVertKDE(fgl, p)
    maxval = topoint(den)
    q = convert(TransformUtils.Quaternion, Euler(maxval[4:6]...))
    newtriad!(vc,p, wTb=Translation(maxval[1:3]...), wRb=Quat(q.s,q.v...), length=0.5)
  end
  if drawlandms
    for l in ll
      # v = getVert(fgl, p)
      den = getVertKDE(fgl, l)
      maxval = topoint(den)
      newpoint!(vc, l, wTb=Translation(maxval[1:3]...))
    end
  end

  visualizetriads!(vc)
  nothing
end

# should be using Twan's code here
function updaterealtime!{T}(vc::VisualizationContainer{T}, id::T, am::AbstractAffineMap)
  if !haskey(vc.realtime, id)
    newtriad!(vc.realtime, vc.rttfs, id, wRb=Quat(am.m.w,am.m.x,am.m.y,am.m.z), wTb=Translation(am.v...), length=0.6)
    vc.models[:realtime] = Visualizer(vc.realtime, 9999)
  else
    vc.rttfs[id] = am
  end
  nothing
end

function visualizerealtime(vc::VisualizationContainer)
  DrakeVisualizer.draw(vc.models[:realtime], vc.rttfs)
  nothing
end


function visualizeDensityMesh!(vc::VisualizationContainer, fgl::FactorGraph, lbl::Symbol; levels=3, meshid::Int=2)

  pl1 = marginal(getVertKDE(fgl,lbl),[1;2;3])

  gg = (x, a=0.0) -> evaluateDualTree(pl1, ([x[1];x[2];x[3]]')')[1]-a

  x = getKDEMax(pl1)
  maxval = gg(x)

  vv = getKDERange(pl1)
  lower_bound = Vec(vec(vv[:,1])...)
  upper_bound = Vec(vec(vv[:,2])...)

  levels = linspace(0.0,maxval,levels+2)

  MD = []
  for val in levels[2:(end-1)]
    meshdata = GeometryData(contour_mesh(x -> gg(x,val), lower_bound, upper_bound))
    meshdata.color = RGBA( val/(1.5*maxval),0.0,1.0,val/(1.5*maxval))
    push!(MD, meshdata)
  end
  mptr = Any(MD)
  vc.meshes[lbl] = mptr
  Visualizer(mptr, meshid) # meshdata
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











#
