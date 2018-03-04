# Visualize with Director
addprocs(2)

using TransformUtils
using KernelDensityEstimate
# using IncrementalInference
using RoME
using Caesar
using GeometryTypes
using DrakeVisualizer
using ColorTypes: RGBA
# using MeshIO, FileIO
using CoordinateTransformations
using Rotations
using Distributions

# # start visualizer and models
# vc = startdefaultvisualization(draworigin=false)
# sleep(2.0)
# rovt(vc)
# am = Translation(0,0,0) ∘ LinearMap(Rotations.AngleAxis(pi/2,0,0,1.0))
# rovt(vc, am )
#
# # and some default object in the world
# boxdata = GeometryData(HyperRectangle(Vec(0.0,4.0,-0.7), Vec(5.0,5.0,1.4)))
# boxdata.color = RGBA(0.5,0.1,0.0,0.5)
# model = Visualizer(boxdata,100)


# start visualizer with some scene
vc = startdefaultvisualization(draworigin=false)
sleep(3.0)
# defaultscene01!(vc)
sc01 = loadmodel(:scene01)
sc01(vc)

# load necessary data
rovt = loadmodel(:rov)
rovt(vc)

am = Translation(0,0,0.0) ∘ LinearMap(Rotations.AngleAxis(pi/2,0,0,1.0))
rovt(vc, am )


l1p = [0.0; 4.0; 0.7]
featuredata = GeometryData(HyperSphere(Point(l1p...), 0.10) )
featuredata.color = RGBA(0.5, 1.0, 0.0, 0.7)
model = Visualizer(featuredata,101)

# l1p = [0.0; 4.0; 0.7]
# featuredata = GeometryData(HyperSphere(Point(l1p...), 0.1) )
# featuredata.color = RGBA(0.0, 0.0, 1.0, 0.6)
# model = Visualizer(featuredata,102)



# @async begin
#   for t in linspace(0,1,50)
#     x = -t*0.7
#     phi = pi/4.0*t
#     am = Translation(x,0,0) ∘ LinearMap(Rotations.AngleAxis(pi/2,0,0,1.0)) ∘ LinearMap( Rotations.AngleAxis(phi,1.0,0,0))
#     rovt(vc, am )
#     sleep(0.05)
#   end
# end


# true position
gt = Dict{Symbol, Vector{Float64}}()
gt[:l1] = [4.0;0.0;-0.7]


# okay build the graph
fg = initfg()
N = 200

initCov = 0.01*eye(6)
[initCov[i,i] = 0.001 for i in 4:6];
odoCov = deepcopy(initCov)

println("Adding PriorPose3 and :x1 to graph...")

initp = SE3(zeros(3),TransformUtils.AngleAxis(pi/2,[0;0;1.0]))
initPosePrior = PriorPose3( MVNormal(veeEuler(initp), initCov) )
X= getSample(initPosePrior,N)[1]
v0 = addNode!(fg, :x1,  X,  N=N)
f1  = addFactor!(fg,[v0], initPosePrior)

rangecov, bearingcov=3e-3, 3e-3
println("Adding :l1 LinearRangeBearingElevation to graph...")
addLinearArrayConstraint(fg, (4.0, 0.0), :x1, :l1, rangecov=rangecov,bearingcov=bearingcov)
visualizeDensityMesh!(vc, fg, :l1, meshid=2)

# @async begin
for t in linspace(0,1,50)
  x = -t*0.7
  phi = pi/4.0*t
  am = Translation(x,0,0) ∘ LinearMap(Rotations.AngleAxis(pi/2,0,0,1.0)) ∘ LinearMap( Rotations.AngleAxis(phi,1.0,0,0))
  rovt(vc, am )
  sleep(0.02)
end
# end


println("Adding :x2 pose and to graph...")
tf = [0.0;0.7;0.0;pi/4;0.0;0.0]
odo = Pose3Pose3(MvNormal(tf, odoCov))
addOdoFG!(fg, odo , N=N)

visualizeallposes!(vc, fg, drawlandms=false)
addLinearArrayConstraint(fg, (4.0, 0.0), :x2, :l2, rangecov=rangecov,bearingcov=bearingcov)

visualizeDensityMesh!(vc, fg, :l2, meshid=2)
sleep(5.0)

# and a third seen from both poses
addLinearArrayConstraint(fg, (4.0, 0.0), :x1, :l3, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (4.0, 0.0), :x2, :l3, rangecov=rangecov,bearingcov=bearingcov)

# visualizeDensityMesh!(vc, fg, :l2, meshid=3)


batchSolve(fg)
visualize(fg, vc, drawlandms=false, densitymeshes=[:l3;:l2], N=N)
visualizeDensityMesh!(vc, fg, :x2, meshid=3)

#
