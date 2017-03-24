addprocs(2)
# Visualize SONAR with Director

using TransformUtils
using KernelDensityEstimate
using IncrementalInference
using RoME
using Caesar
using DrakeVisualizer, GeometryTypes
using ColorTypes: RGBA
using MeshIO, FileIO
using CoordinateTransformations
using Rotations

# load necessary data
rovt = loadmodel(:rov)

# true position
gt = Dict{Symbol, Vector{Float64}}()
gt[:l1] = [4.0;0.0;-0.7]

# start visualizer and models
vc = startdefaultvisualization()
rovt(vc)


# and some default object in the world
boxdata = GeometryData(HyperRectangle(Vec(4.0,0.0,-0.7), Vec(5.0,5.0,1.4)))
boxdata.color = RGBA(0.5,0.1,0.0,0.5)
model = Visualizer(boxdata,100)

# l1p = [4.0; 0; 0.7]
# featuredata = GeometryData(HyperSphere(Point(l1p...), 0.15) )
# featuredata.color = RGBA(0.0, 1.0, 0.0, 1.0)
# model = Visualizer(featuredata,101)

# l1p = [4.0; 0; -0.7]
# featuredata = GeometryData(HyperSphere(Point(l1p...), 0.15) )
# featuredata.color = RGBA(0.0, 0.0, 1.0, 1.0)
# model = Visualizer(featuredata,102)




@async begin
  rho = 5.0
  pnt = (4.5,0.0)
  for t in linspace(0,1,100)
    the = t*(pi/2+pi/6)
    x = -rho*cos(the)+pnt[1]
    y = -rho*sin(the)
    rovt(vc, Translation(x,y,0) ∘ LinearMap(Rotations.AngleAxis(the,0,0,1.0)) )
    # draw(modelrov, [Translation(x, y, 0.25) ∘ LinearMap(Rotations.AngleAxis(pi+the,0,0,1.0))])
    sleep(0.05)
  end
  for t in linspace(0,1,25)
    the = (pi/2+pi/6)
    themax = pi/2+pi/6
    xmax = -rho*cos(themax)+pnt[1]
    ymax = -rho*sin(themax)
    z = t
    rovt(vc, Translation(xmax,ymax,-z) ∘ LinearMap(Rotations.AngleAxis(themax,0,0,1.0)) )
    # draw(modelrov, [Translation(xmax, ymax, 0.25-z) ∘ LinearMap(Rotations.AngleAxis(pi+themax,0,0,1.0))])
    sleep(0.05)
  end
  for t in linspace(1,0.25,75)
    the = t*(pi/2+pi/6)
    themax = pi/2+pi/6
    x = -rho*cos(the)+pnt[1]
    y = -rho*sin(the)
    xmax = -rho*cos(themax)+pnt[1]
    ymax = -rho*sin(themax)
    rovt(vc, Translation(x,y,-1.0) ∘ LinearMap(Rotations.AngleAxis(the,0,0,1.0)) )
    # draw(modelrov, [Translation(x, y, -0.75) ∘ LinearMap(Rotations.AngleAxis(pi+the,0,0,1.0))])
    sleep(0.05)
  end
end

# covariance parameters
initCov = 0.01*eye(6); [initCov[i,i] = 0.001 for i in 4:6];
odoCov, rangecov, bearingcov = deepcopy(initCov), 3e-4, 2e-4

# okay build the graph
fg = identitypose6fg(initCov=initCov)

println("Adding :l1 LinearRangeBearingElevation to graph...")
addLinearArrayConstraint(fg, (4.12, 0.0), :x1, :l1, rangecov=rangecov,bearingcov=bearingcov)


addOdoFG!(fg, Pose3Pose3(MvNormal([2.0;-4.0;0.0;0.0;0.0;pi/3], odoCov) )
addOdoFG!(fg, Pose3Pose3(MvNormal([2.0;-4.0;0.0;0.0;0.0;pi/3], odoCov) )
visualizeallposes!(vc, fg, drawlandms=false)

rho2, bearing2, elev2 = projectrbe(fg,:x2,gt[:l1])
addLinearArrayConstraint(fg, (rho2, bearing2), :x2, :l1, rangecov=rangecov,bearingcov=bearingcov)

rho3, bearing3, elev3 = projectrbe(fg,:x3,gt[:l1])
addLinearArrayConstraint(fg, (rho3, bearing3), :x3, :l1, rangecov=rangecov,bearingcov=bearingcov)


# solveandvisualize(fg, vc, densitymeshes=[:l1;:x3])


visualizeDensityMesh!(vc, fg, :l1, meshid=2)

addOdoFG!(fg, Pose3Pose3(MvNormal([1.0;2.0;-1.0;0.0;0.0;-pi/6], odoCov) )
visualizeallposes!(vc, fg, drawlandms=false)

rho4, bearing4, elev4 = projectrbe(fg,:x4,gt[:l1])
bearing4 = 0.0
addLinearArrayConstraint(fg, (rho4, bearing4), :x4, :l1, rangecov=rangecov,bearingcov=bearingcov)


# solveandvisualize(fg, vc, densitymeshes=[:l1;:x3;:x4])


addOdoFG!(fg, Pose3Pose3(MvNormal([2.0;4.0;0.0;0.0;0.0;-pi/3], odoCov) )

visualizeallposes!(vc, fg, drawlandms=false)
rho5, bearing5, elev5 = projectrbe(fg,:x5,:l1)
bearing5 = 0.0
addLinearArrayConstraint(fg, (rho5, bearing5), :x5, :l1, rangecov=rangecov,bearingcov=bearingcov)


solveandvisualize(fg, vc, densitymeshes=[:l1;:x5])


visualizeallposes!(vc, fg, drawlandms=false)

visualizeDensityMesh!(vc, fg, :x4, meshid=3)



#
# rovdata = GeometryData(rov )
# rovdata.color = RGBA(0., 1.0, 0.5, 0.3)
# modelrov = Visualizer(rovdata, 99)
# rho = 5.0
# pnt = (4.5,0.0)
# for t in linspace(0,1,100)
#   the = t*(pi/2+pi/6)
#   x = -rho*cos(the)+pnt[1]
#   y = -rho*sin(the)
#   draw(modelrov, [Translation(x, y, 0.25) ∘ LinearMap(Rotations.AngleAxis(pi+the,0,0,1.0))])
#   sleep(0.05)
# end


@show














#
