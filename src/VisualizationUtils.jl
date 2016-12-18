# some drawing utils

# using DrakeVisualizer, CoordinateTransformations, GeometryTypes, Rotations, TransformUtils, ColorTypes


type VisualizationContainer{T}
  model
  triads::Dict{T, Link}
  triadposes::Dict{T, AbstractAffineMap}
  points::Dict{T, HyperSphere}
  pointpositions::Dict{T, Translation}
  # VisualizationContainer() = new()
  # VisualizationContainer(a,b,c) = new{T}(a,b,c, Dict{T, HyperSphere}(), Dict{T, Translation}() )
  # VisualizationContainer(a,b,c,d,e) = new{T}(a,b,c,d,e)
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

function newtriad!{T}(triads::Dict{T,Link}, triadposes::Dict{T, AbstractAffineMap}, id::T;
      wRb::Rotation=Quat(1.,0,0,0),
      wTb::Translation=Translation(0.,0,0),
      width=0.02,
      length=1.0  )
  triads[id] = linkaxestriad()
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

function visualizetriads{T}(triads::Dict{T,Link}, triadposes::Dict{T, AbstractAffineMap})
  triadsmodel = Visualizer(triads)
  DrakeVisualizer.draw(triadsmodel, triadposes)
  triadsmodel
end
visualizetriads(dc::VisualizationContainer) = visualizetriads(dc.triads, dc.triadposes)

function testtriaddrawing()
  triads, trposes = Dict{Int,Link}(), Dict{Int, AbstractAffineMap}()
  newtriad!(triads, trposes, 0)
  newtriad!(triads, trposes, 1,wTb=Translation(2.,0,0),wRb=AngleAxis(pi/4,1.,0,0))
  newtriad!(triads, trposes, 2,wTb=Translation(4.,0,0),wRb=AngleAxis(pi/2,1.,0,0))

  visualizetriads(triads, trposes)
  nothing
end


# create a new Director window with home axis
function startdefaultvisualization(;newwindow=true)
  DrakeVisualizer.new_window()
  triads, trposes = Dict{Symbol, Link}(), Dict{Symbol, AbstractAffineMap}()
  newtriad!(triads, trposes, :origin)
  model = visualizetriads(triads, trposes)
  dc = VisualizationContainer(model, triads, trposes, Dict{Symbol, HyperSphere}(), Dict{Symbol, Translation}())
  return dc
end


function visualizeallposes!(dc::VisualizationContainer, fgl::FactorGraph)
  po,ll = ls(fgl)
  for p in po
    # v = getVert(fgl, p)
    den = getVertKDE(fgl, p)
    maxval = getKDEMax(den)
    q = convert(TransformUtils.Quaternion, Euler(maxval[4:6]...))
    newtriad!(dc,p, wTb=Translation(maxval[1:3]...), wRb=Quat(q.s,q.v...))
  end
  visualizetriads(dc)
  nothing
end



# testtriaddrawing()











#
