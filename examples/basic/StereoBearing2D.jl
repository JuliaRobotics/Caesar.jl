# a 2D triangulation example
# this example uses a factor graph to represent the data
# Factor graphs can be used to construct either fixed lag smoothing or full blown SLAM solutions.

# Load required libraries
using Caesar, RoMEPlotting
# using Distributions
# using ApproxManifoldProducts

## Environment setup -- in Juno you can run cells with Alt+Enter

# true pose position (assume hidden)  - [X Y θ]
p0 = [0.0;0.0;0.0]

# noise level for Gaussian assumption -- see https://github.com/JuliaStats/Distributions.jl/issues/584
pcov = Matrix(Diagonal([0.01; 0.01; 0.05].^2))

# true landmark positions
l0 = [2.0;0.15]
l1 = [2.0;-0.15]

d0 = l0-p0[1:2]
m0 = atan(d0[2],d0[1])

d1 = l1-p0[1:2]
m1 = atan(d1[2],d1[1])



mstd = 0.1

## build the factor graph

fg = initfg()

# add two beacons
addVariable!(fg, :l0, Point2, labels=["BEACON"])
# addFactor!(fg, [:l0], Prior(MvNormal(l0, lcov)))

addVariable!(fg, :l1, Point2, labels=["BEACON"])
# addFactor!(fg, [:l1], Prior(MvNormal(l1, lcov)))

addFactor!(fg, [:l0; :l1], Point2Point2(MvNormal(l1-l0, [0.01 0; 0 0.01].^2)))


# add unknown pose location
addVariable!(fg, :x0, Pose2, labels=["POSE"])
addFactor!(fg, [:x0], PriorPose2(MvNormal(p0, pcov)))

# add two bearing only measurements
addFactor!(fg, [:x0; :l0], Pose2Point2BearingRange(Normal(m0, mstd), Rayleigh(2.0)))
addFactor!(fg, [:x0; :l1], Pose2Point2BearingRange(Normal(m1, mstd), Rayleigh(2.0)))

# addFactor!(fg, [:l1], PartialPrior(Normal(0.0,0.1), (1,)))


# # add unknown pose location
# addVariable!(fg, :x1, Pose2, labels=["POSE"])
# addFactor!(fg, [:x1], PriorPose2(MvNormal(p1, pcov)))
#
# # add two bearing only measurements
# addFactor!(fg, [:x1; :l0], Pose2Point2Bearing(Normal(m2, mstd)))
# addFactor!(fg, [:x1; :l1], Pose2Point2Bearing(Normal(m3, mstd)))
#
#
# addFactor!(fg, [:x0; :x1], Pose2Pose2(MvNormal([0.0;1.0;0.0], [0.01 0 0; 0 0.01 0; 0 0 0.01].^2)))



drawGraph(fg, show=true)

# ensureAllInitialized!(fg)
# setValKDE!(fg, :l0, AMP.manikde!(randn(2,100), Point2().manifolds))
# setValKDE!(fg, :l1, AMP.manikde!(randn(2,100), Point2().manifolds))

# solve the system -- remember first runs are slow (just in time compile code)
solveTree!(fg)


## Plot values to see result


Gadfly.set_default_plot_size(30cm,20cm)
plotPose(Pose2(), [getKDE(fg, :x0)]) #, axis=[-0.5 2.0; -0.5 1.5])

# plotPose(Pose2(), [getKDE(fg, :x1)]) #, axis=[-0.5 2.0; -0.5 1.5])


plotKDE(fg, [:l0; :l1],dims=[1;2],levels=3) # ,axis=[0 2.0; 0.0 1.5])






## Building a test



fg = initfg()

addVariable!(fg, :x0, Pose2)
addFactor!(fg, [:x0], PriorPose2(MvNormal(zeros(3),Matrix(Diagonal(0.001.*ones(3))))))

addVariable!(fg, :l0, Point2)
addFactor!(fg, [:l0], PriorPoint2(MvNormal([1.0;0.0],Matrix(Diagonal(0.001.*ones(2))))))
addFactor!(fg, [:x0;:l0], Pose2Point2Bearing(Normal(0, 0.01)))


addVariable!(fg, :l1, Point2)
addFactor!(fg, [:l1], PriorPoint2(MvNormal([0.0;1.0],Matrix(Diagonal(0.001.*ones(2))))))
addFactor!(fg, [:x0;:l1], Pose2Point2Bearing(Normal(pi/2, 0.01)))




fg = initfg()

addVariable!(fg, :x0, Pose2)
addFactor!(fg, [:x0], PriorPose2(MvNormal(zeros(3),Matrix(Diagonal(0.001.*ones(3))))))

addVariable!(fg, :l0, Point2)
# addFactor!(fg, [:l0], PriorPoint2(MvNormal([1.0;0.0],Matrix(Diagonal(0.001.*ones(2))))))
addFactor!(fg, [:x0;:l0], Pose2Point2Bearing(Normal(0, 0.01)))


addVariable!(fg, :l1, Point2)
# addFactor!(fg, [:l1], PriorPoint(MvNormal([0.0;1.0],Matrix(Diagonal(0.001.*ones(2))))))
addFactor!(fg, [:x0;:l1], Pose2Point2Bearing(Normal(pi/2, 0.01)))

addFactor!(fg, [:l0;:l1], Point2Point2(MvNormal([-2.0; 2.0], Matrix(Diagonal(0.001.*ones(2))))))




## util development








## more debugging



stuff = localProduct(fg, :x0)

plotKDE(stuff[2],dims=[1;2],levels=3, c=["red","green","blue"])






stuff = localProduct(fg, :l0)

plotKDE([stuff[1];stuff[2]],dims=[1;2],levels=3, c=["black","red","green","blue"])




pcm = [marginal(stuff[2][1],[3]);marginal(stuff[2][2],[3])]
AMP.plotKDECircular(pcm)


np = manifoldProduct(stuff[2], Pose2().manifolds)


AMP.plotKDECircular(marginal(np,[3]))


## more direct calculation

pts0 = approxConv(fg, :x0l0f1, :x0)
X0 = manikde!(pts0, Pose2().manifolds)

plotPose(Pose2(), [X0])

pts1 = approxConv(fg, :x0l1f1, :x0)
X1 = manikde!(pts1, Pose2().manifolds)

plotPose(Pose2(), [X1])






## Major error here

ptsl0 = approxConv(fg, :x0l0f1, :l0)
L0 = manikde!(ptsl0, Point2().manifolds)
plotKDE(L0, levels=3)


stuff = IIF.localProduct(fg, :l0)

plotKDE([stuff[2];stuff[1]])

## more debugging below



pts0_0 = approxConv(fg, :x0l0f1, :l0)
L00 = manikde!(pts0_0, Point2().manifolds)

plotKDE(L00, levels=3, axis=[-0.5 2; -0.5 1.5])






pts0_1 = approxConv(fg, :x0l1f1, :l1)
L01 = manikde!(pts0_1, Point2().manifolds)

plotKDE(L01, levels=3, axis=[-0.5 2; -0.5 1.5])


#

getSolverParams(fg).drawtree=true
getSolverParams(fg).showtree=true
tree = solveTree!(fg)


spyCliqMat(tree,:l0)

writeGraphPdf(fg, show=true)




stuff = IIF.treeProductUp(fg, tree, :l0, :l0)

Gadfly.plot(x=stuff[1][1,:], y=stuff[1][2,:], Geom.histogram2d)




## Util development







##
