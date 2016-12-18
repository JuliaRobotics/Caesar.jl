# some drawing utils

using DrakeVisualizer, CoordinateTransformations, GeometryTypes, Rotations, TransformUtils, ColorTypes

function linkaxestriad(;width=0.02,length=1.0)
  Xbox = GeometryData(HyperRectangle(Vec(0.,-width,-width), Vec(length,width,width)))
  Xbox.color = RGBA(1., 0, 0, 0.5)
  Ybox = GeometryData(HyperRectangle(Vec(-width,0.0,-width), Vec(width,length,width)))
  Ybox.color = RGBA(0., 1, 0, 0.5)
  Zbox = GeometryData(HyperRectangle(Vec(-width,-width,0.0), Vec(width,width,length)))
  Zbox.color = RGBA(0., 0, 1, 0.5)
  Link([Xbox;Ybox;Zbox])
end

function settriadpose!(triadposes::Dict{Int, AbstractAffineMap}, id::Int, wTb::Translation, wRb::Rotation)
  triadposes[id] = wTb ∘ LinearMap(wRb)
  nothing
end
settriadpose!(triadposes::Dict{Int, AbstractAffineMap}, id::Int, wTb::Vector{Float64}, wQb::TransformUtils.Quaternion) =
    updatetriad!(triadposes, id, Translation(wTb...), Quat(wQb.s, wQb.v...))

function newtriad!(triads::Dict{Int,Link}, triadposes::Dict{Int, AbstractAffineMap}, id::Int;
      wRb::Rotation=Quat(1.,0,0,0),
      wTb::Translation=Translation(0.,0,0),
      width=0.02,
      length=1.0  )
  triads[id] = linkaxestriad()
  settriadpose!(triadposes, id, wTb, wRb)
  nothing
end

function updatetriadposes!(model, triadposes::Dict{Int, AbstractAffineMap})
  draw(model, triadposes)
  nothing
end

function visualizetriads(triads::Dict{Int,Link}, triadposes::Dict{Int, AbstractAffineMap})
  triadsmodel = Visualizer(triads)
  updatetriadposes!(triadsmodel, triadposes)
  nothing
end

function testtriaddrawing()
  triads, trposes = Dict{Int,Link}(), Dict{Int, AbstractAffineMap}()
  newtriad!(triads, trposes, 0)
  newtriad!(triads, trposes, 1,wTb=Translation(2.,0,0),wRb=AngleAxis(pi/4,1.,0,0))
  newtriad!(triads, trposes, 2,wTb=Translation(4.,0,0),wRb=AngleAxis(pi/2,1.,0,0))

  visualizetriads(triads, trposes)
  nothing
end

DrakeVisualizer.new_window();

testtriaddrawing()











#
