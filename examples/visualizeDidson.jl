# Visualize Didson with Director

using TransformUtils, Gadfly
using KernelDensityEstimate
using IncrementalInference
using RoME
using Caesar

using Distributions
# using Base.Test


# start visualizer
vc = startdefaultvisualization()


# okay build the graph
fg = initfg()
N = 100

initCov = 0.01*eye(6)
[initCov[i,i] = 0.001 for i in 4:6];
odoCov = deepcopy(initCov)

println("Adding PriorPose3 and :x1 to graph...")
X, pts = 0.01*randn(6,N), zeros(3,N);

v0 = addNode!(fg, :x1,  X,  N=N)
initPosePrior = PriorPose3(SE3(0), initCov)
f1  = addFactor!(fg,[v0], initPosePrior)

rangecov, bearingcov=3e-5, 3e-5
println("Adding :l1 LinearRangeBearingElevation to graph...")
addLinearArrayConstraint(fg, (4.0, 0.0), :x1, :l1, rangecov=rangecov,bearingcov=bearingcov)

println("Adding :x2 pose and to graph...")
odo = SE3([0.0;0.7;0.0], Euler(pi/4,0.0,0.0) )
addOdoFG!(fg, Pose3Pose3(deepcopy(odo), odoCov) )
addLinearArrayConstraint(fg, (4.0, 0.0), :x2, :l2, rangecov=rangecov,bearingcov=bearingcov)

# and a third seen from both poses
addLinearArrayConstraint(fg, (4.0, 0.0), :x1, :l3, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (4.0, 0.0), :x2, :l3, rangecov=rangecov,bearingcov=bearingcov)



# draw while solving[1]==true
solving = [true]
@async begin
  while solving[1]
    println(".")
    visualizeallposes!(vc, fg)
    visualizeDensityMesh!(vc, fg, :x1, meshid=2)
    visualizeDensityMesh!(vc, fg, :x2, meshid=3)
    visualizeDensityMesh!(vc, fg, :l3, meshid=4)
    sleep(1)
  end
end

# solve
tree = wipeBuildNewTree!(fg)
@time inferOverTree!(fg, tree)
solving[1]=false;


#
