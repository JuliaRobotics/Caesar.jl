# some drawing utils

using DrakeVisualizer, CoordinateTransformations, GeometryTypes, Rotations, TransformUtils, ColorTypes


type VisualizationContainer{T}
  models::Dict{T, Visualizer}
  triads::Dict{T, Link}
  triadposes::Dict{T, AbstractAffineMap}
  meshes::Dict{T, Link}
  # pointpositions::Dict{T, Translation}
end


function linkaxestriad(;width=0.02,length=1.0)
  Xbox = GeometryData(HyperRectangle(Vec(0.,-width,-width), Vec(length,width,width)))
  Xbox.color = RGBA(1., 0, 0, 0.5)
  Ybox = GeometryData(HyperRectangle(Vec(-width,0.0,-width), Vec(width,length,width)))
  Ybox.color = RGBA(0., 1, 0, 0.5)
  Zbox = GeometryData(HyperRectangle(Vec(-width,-width,0.0), Vec(width,width,length)))
  Zbox.color = RGBA(0., 0, 1, 0.5)
  Link([Xbox;Ybox;Zbox])
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

function newtriad!{T}(triads::Dict{T,Link}, triadposes::Dict{T, AbstractAffineMap}, id::T;
      wRb::Rotation=Quat(1.,0,0,0),
      wTb::Translation=Translation(0.,0,0),
      width=0.02,
      length=1.0  )
  triads[id] = linkaxestriad(width=width, length=length)
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
  dc.triads[id] = Link([pt])
  setpointpose!(dc.triadposes, id, wTb)
  nothing
end

# function visualizetriads{T}(triads::Dict{T,Link}, triadposes::Dict{T, AbstractAffineMap})
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
  # triads, trposes = Dict{Int,Link}(), Dict{Int, AbstractAffineMap}()
  vc = VisualizationContainer(nothing,Dict{Int,Link}(), Dict{Int, AbstractAffineMap}(), Dict{Int, HomogenousMesh}())
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
  triads, trposes, meshes = Dict{Symbol, Link}(), Dict{Symbol, AbstractAffineMap}(), Dict{Symbol, Link}()
  draworigin ? newtriad!(triads, trposes, :origin) : nothing
  dc = VisualizationContainer(Dict{Symbol, Visualizer}(), triads, trposes, meshes)
  visualizetriads!(dc)
  # model = visualizetriads(triads, trposes)
  return dc
end


function visualizeallposes!(vc::VisualizationContainer, fgl::FactorGraph; drawlandms::Bool=true)
  po,ll = ls(fgl)
  for p in po
    # v = getVert(fgl, p)
    den = getVertKDE(fgl, p)
    maxval = getKDEMax(den)
    q = convert(TransformUtils.Quaternion, Euler(maxval[4:6]...))
    newtriad!(vc,p, wTb=Translation(maxval[1:3]...), wRb=Quat(q.s,q.v...), length=0.5)
  end
  if drawlandms
    for l in ll
      # v = getVert(fgl, p)
      den = getVertKDE(fgl, l)
      maxval = getKDEMax(den)
      newpoint!(vc, l, wTb=Translation(maxval[1:3]...))
    end
  end

  visualizetriads!(vc)
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
  mptr = Link(MD)
  vc.meshes[lbl] = mptr
  Visualizer(mptr, meshid) # meshdata
  nothing
end




# DrakeVisualizer.new_window()
# vctest = testtriaddrawing()











#
