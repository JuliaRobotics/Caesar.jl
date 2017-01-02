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
N = 200


initCov = 0.01*eye(6)
[initCov[i,i] = 0.001 for i in 4:6];
odoCov = deepcopy(initCov)

println("Adding PriorPose3 to graph...")
X, pts = 0.01*randn(6,N), zeros(3,N);

v0 = addNode!(fg, :x1,  X,  N=N)
initPosePrior = PriorPose3(SE3(0), initCov)
f1  = addFactor!(fg,[v0], initPosePrior)


println("Adding LinearRangeBearingElevation to graph...")
meas = LinearRangeBearingElevation((3.0,3e-5),(0.0,3e-5)) #, elev=Uniform(0.0,0.25))
# fp! = WrapParam{reuseLBRA}(zeros(3), zeros(6), zeros(3), reuseLBRA(0))
# for i in 1:N
# 	project!(meas, X, pts, i, fp!)
# end

pts = getVal(fg,:x1) + meas
v1 = addNode!(fg, :l1, pts, N=N)
addFactor!(fg,[v0;v1],meas)


visualizeallposes!(vc, fg)
visualizeDensityMesh!(vc, fg, :l1)



odo = SE3([0.0;0.7;0.0], Euler(pi/4,0.0,0.0) )
# v, f = addPose3Pose3(fg, "x3", deepcopy(odo), odoCov, N=N)
addOdoFG!(fg, Pose3Pose3(deepcopy(odo), odoCov) )



meas2 = LinearRangeBearingElevation((3.0,3e-5),(0.0,3e-5))
pts = getVal(fg,:x2) + meas2
# v2 = addNode!(fg, :l2, pts, N=N)
addFactor!(fg,[getVert(fg, :x2); v1],meas2)



# solve
tree = wipeBuildNewTree!(fg)
inferOverTree!(fg, tree)

visualizeallposes!(vc, fg)
visualizeDensityMesh!(vc, fg, :l1)

# visualizeDensityMesh!(vc, fg, :x2)









# separate test


# okay build the graph
fg = initfg()
N = 200

initCov = 0.01*eye(6)
[initCov[i,i] = 0.001 for i in 4:6];
odoCov = deepcopy(initCov)

println("Adding PriorPose3 to graph...")
X, pts = 0.01*randn(6,N), zeros(3,N);

v0 = addNode!(fg, :x1,  X,  N=N)
initPosePrior = PriorPose3(SE3(0), initCov)
f1  = addFactor!(fg,[v0], initPosePrior)


println("Adding LinearRangeBearingElevation to graph...")
meas = LinearRangeBearingElevation((4.0,3e-5),(0.0,3e-5))
fp! = WrapParam{reuseLBRA}(zeros(3), zeros(6), zeros(3), reuseLBRA(0))
for i in 1:N
	project!(meas, X, pts, i, fp!)
end
v1 = addNode!(fg, :l1, pts, N=N)
addFactor!(fg,[v0;v1],meas)


odo = SE3([0.0;0.0;-1.0], Euler(pi/4.0,0.0, 0.0) )
# v, f = addPose3Pose3(fg, "x3", deepcopy(odo), odoCov, N=N)
addOdoFG!(fg, Pose3Pose3(deepcopy(odo), odoCov) )


meas2 = LinearRangeBearingElevation((4.0,3e-5),(0.0,3e-5))
addFactor!(fg,[getVert(fg, :x2);v1],meas2)


visualizeallposes!(vc, fg)
visualizeDensityMesh!(vc, fg, :l1)



# solve
tree = wipeBuildNewTree!(fg)
inferOverTree!(fg, tree)



visualizeallposes!(vc, fg)
visualizeDensityMesh!(vc, fg, :l1)


#
