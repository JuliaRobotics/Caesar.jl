# Visualize SONAR with Director

using TransformUtils
using KernelDensityEstimate
using IncrementalInference
using RoME
using Caesar
using DrakeVisualizer, GeometryTypes
using ColorTypes: RGBA

# start visualizer
vc = startdefaultvisualization()

# and some default object in the world
box = HyperRectangle(Vec(4.0,0.0,-0.7), Vec(2.0,2.0,1.4))
model = Visualizer(box,100)

l1p = [4.0; 0; 0.7]
featuredata = GeometryData(HyperSphere(Point(l1p...), 0.08) )
featuredata.color = RGBA(0., 1, 0, 0.9)
model = Visualizer(featuredata,101)

# covariance parameters
initCov = 0.01*eye(6); [initCov[i,i] = 0.001 for i in 4:6];
odoCov, rangecov, bearingcov = deepcopy(initCov), 3e-4, 2e-4

# okay build the graph
fg = identitypose6fg(initCov=initCov)



println("Adding :l1 LinearRangeBearingElevation to graph...")
addLinearArrayConstraint(fg, (4.12, 0.0), :x1, :l1, rangecov=rangecov,bearingcov=bearingcov)


addOdoFG!(fg, Pose3Pose3(SE3([2.0;-4.0;0.0], Euler(0.0,0.0,pi/3) ), odoCov) )
addOdoFG!(fg, Pose3Pose3(SE3([2.0;-4.0;0.0], Euler(0.0,0.0,pi/3) ), odoCov) )
visualizeallposes!(vc, fg)

# addLinearArrayConstraint(fg, (4.12, 0.0), :x2, :l1, rangecov=rangecov,bearingcov=bearingcov)

x3p = getKDEMax(getVertKDE(fg,:x3))[1:3]
@show rho3 = norm(l1p-x3p)
addLinearArrayConstraint(fg, (rho3, 0.0), :x3, :l1, rangecov=rangecov,bearingcov=bearingcov)

visualizeDensityMesh!(vc, fg, :l1, meshid=2)

solveandvisualize(fg, vc, densitymeshes=[:l1;:x3])


using MeshIO, FileIO

using CoordinateTransformations

rov = load("/home/dehann/Downloads/rov2.obj")

rovdata = GeometryData(rov )
rovdata.color = RGBA(0., 1.0, 0.5, 0.3)
modelrov = Visualizer(rovdata, 99)
for y in linspace(0,1,100)
  draw(modelrov, [Translation(0, -y, 0)])
  sleep(0.01)
end
#
