# ROV visualization

# using MeshIO, FileIO
# using CoordinateTransformations
# using Rotations
# using ColorTypes: RGBA

abstract DrawModel <: Function

# Modified ROV model from GrabCAD
# http://grabcad.com/library/rov-7
type DrawROV <: DrawModel
  data
  visid::Int
  symbol::Symbol
  offset::AffineMap
end

function loadmodel(model::Symbol=:rov;
    color=RGBA(0., 1.0, 0.5, 0.3),
    offset = Translation(0.,0,0) ∘ LinearMap(Rotations.Quat(1.0,0,0,0))  )
  #
  if model==:rov
    @show file = joinpath(Pkg.dir("Caesar"), "data", "models", "rov2.obj")
    rov = load(file)
    rovdata = GeometryData(rov)
    rovdata.color = color
    offset = Translation(-0.5,0,0.25) ∘ LinearMap(Rotations.Quat(0,0,0,1.0))
    return DrawROV(rovdata, 99, :rov, offset)
  else
    error("Don't recognize requested $(string(model)) model.")
  end
end

#syntax support lost in 0.5, but see conversation at
# function (dmdl::DrawModel)(vc::VisualizationContainer)
# issue  https://github.com/JuliaLang/julia/issues/14919

function (dmdl::DrawROV)(vc::VisualizationContainer,
        am::AffineMap  )
  #
  # mam = am.m
  # if typeof(am.m)==Rotations.AngleAxis{Float64}
  #   mam = convert(Rotations.Quat, am.m)
  # end
  # res = SE3(am.v[1:3],Quaternion(mam.w,[mam.x;mam.y;mam.z]))*SE3(dmdl.offset.v[1:3],Quaternion(dmdl.offset.m.w,[dmdl.offset.m.x;dmdl.offset.m.y;dmdl.offset.m.z]))
  # q = convert(Quaternion, res.R)
  # DrakeVisualizer.draw(vc.models[dmdl.symbol], [Translation(res.t...) ∘ LinearMap(Rotations.Quat(q.s,q.v...))])

  DrakeVisualizer.draw(vc.models[dmdl.symbol], [am ∘ dmdl.offset])
  nothing
end
function (dmdl::DrawROV)(vc::VisualizationContainer,
        t::Translation,
        R::Rotation  )
  #
  dmdl(vc, t ∘ LinearMap(R))
  nothing
end
function (dmdl::DrawROV)(vc::VisualizationContainer)
  vc.models[dmdl.symbol] = Visualizer(dmdl.data, dmdl.visid)
  dmdl(vc, Translation(0.,0,0) ∘ LinearMap(Rotations.Quat(1.0,0,0,0)) )
end


function defaultscene01!(vc::VisualizationContainer; meshid=100)
  println("Starting here")
  ln = []
  boxdata = GeometryData(HyperRectangle(Vec(0.1,4.1,-0.7), Vec(5.0,5.0,1.4)))
  boxdata.color = RGBA(0.5,0.1,0.0,0.5)
  push!(ln, boxdata)
  # # model = Visualizer(boxdata,100)
  #
  println("going here")
  # l1p = [0.0; 4.0; 0.7]
  # featuredata = GeometryData(HyperSphere(Point(l1p...), 0.10) )
  # featuredata.color = RGBA(0.5, 1.0, 0.0, 0.7)
  # push!(ln, featuredata)
  # # model = Visualizer(featuredata,101)
  #
  # # l1p = [0.0; 4.0; 0.7]
  # # featuredata = GeometryData(HyperSphere(Point(l1p...), 0.1) )
  # # featuredata.color = RGBA(0.0, 0.0, 1.0, 0.6)
  # # model = Visualizer(featuredata,102)
  println("going to draw")
  vc.meshes[:scene] = Link(ln)
  vc.models[:scene] = Visualizer(vc.meshes[:scene],meshid)
  nothing
end




# t∈[0,1], about
function parameterizeArcAffineMap(t, as::ArcPointsRangeSolve; initrot::Rotation=Rotations.Quat(1.0,0,0,0))
  the = t*as.angle
  rot = Rotations.AngleAxis(the, as.axis...)
  dp = as.x1-as.center
  arc = LinearMap(rot) ∘ Translation(dp...)

  cent = Translation(as.center)
  return cent ∘ arc ∘ LinearMap(initrot)
end


function animatearc(vc::VisualizationContainer,
            drmodel::DrawModel,
            as::ArcPointsRangeSolve;
            N::Int=100,
            delaytime::Float64=0.05,
            initrot::Rotation=Rotations.Quat(1.0,0,0,0)   )
  #
  for t in linspace(0,1,N)
    am = parameterizeArcAffineMap(t, as, initrot=initrot )
    drmodel(vc, am )
    sleep(delaytime)
  end
  nothing
end

# as = ArcPointsRangeSolve(
#       [0.0;0;0.0],
#       [0.0;6.0;6.0],
#       [0.0;12.0;0.0], 6.0)
# findaxiscenter!(as)
# # @show as.center, as.axis, as.angle, as.r
# animatearc(vc, rovt, as, initrot=Rotations.AngleAxis(pi/2,0,0,1.0))
#
#
# as = ArcPointsRangeSolve(
#       [2.5;-1;0.0],
#       [-4.5;4.0;0.0],
#       [9.5;6.0;0.0], 7.0)
# findaxiscenter!(as)
# # @show as.center, as.axis, as.angle, as.r
# animatearc(vc, rovt, as, initrot=Rotations.AngleAxis(pi/2,0,0,1.0))
#





#
