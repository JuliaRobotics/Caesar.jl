# Visualize Didson with Director

using TransformUtils, Gadfly
using KernelDensityEstimate
using IncrementalInference
using RoME
using Caesar
# using Base.Test


# using CoordinateTransformations, GeometryTypes, DrakeVisualizer, ColorTypes


#
# function visualizeDensityMesh!(vc::VisualizationContainer, fgl::FactorGraph, lbl::Symbol; levels=3)
#
#   pl1 = marginal(getVertKDE(fgl,lbl),[1;2;3])
#
#   gg = (x, a=0.0) -> evaluateDualTree(pl1, ([x[1];x[2];x[3]]')')[1]-a
#
#   x = getKDEMax(pl1)
#   maxval = gg(x)
#
#   vv = getKDERange(pl1)
#   lower_bound = Vec(vec(vv[:,1])...)
#   upper_bound = Vec(vec(vv[:,2])...)
#
#   levels = linspace(0.0,maxval,levels+2)
#
#   MD = []
#   for val in levels[2:(end-1)]
#     meshdata = GeometryData(contour_mesh(x -> gg(x,val), lower_bound, upper_bound))
#     meshdata.color = RGBA( val/(1.5*maxval),0.0,1.0,val/(1.5*maxval))
#     push!(MD, meshdata)
#   end
#   mptr = Link(MD)
#   vc.meshes[lbl] = mptr
#   Visualizer(mptr, 2) # meshdata
#   nothing
# end
#



# okay build the graph
fg = initfg()
N = 300

# start visualizer
vc = startdefaultvisualization()


initCov = 0.01*eye(6)
[initCov[i,i] = 0.001 for i in 4:6];
odoCov = deepcopy(initCov)

println("Adding PriorPose3 to graph...")
X, pts = 0.01*randn(6,N), zeros(3,N);

v0 = addNode!(fg, :x1,  X,  N=N)
initPosePrior = PriorPose3(SE3(0), initCov)
f1  = addFactor!(fg,[v0], initPosePrior)


println("Adding LinearRangeBearingElevation to graph...")
meas = LinearRangeBearingElevation((3.0,3e-4),(0.0,3e-4))
for i in 1:N
	project!(meas, X, pts, i)
end
v1 = addNode!(fg, :l1, pts, N=N)
addFactor!(fg,[v0;v1],meas)


visualizeallposes!(vc, fg)




# solve
tree = wipeBuildNewTree!(fg)
[inferOverTree!(fg, tree) for i in 1:1];


visualizeDensityMesh!(vc, fg, :l1)


#
visualizeDensityMesh!(vc, fg, :l1)





odo = SE3([3.0;3.0;0.0], Euler(0.0,0.0, -pi/2.0) )
# v, f = addPose3Pose3(fg, "x3", deepcopy(odo), odoCov, N=N)
addOdoFG!(fg, Pose3Pose3(deepcopy(odo), odoCov) )



meas2 = LinearRangeBearingElevation((3.0,3e-4),(0.0,3e-4))
addFactor!(fg,[getVert(fg, :x2);v1],meas2)











#
