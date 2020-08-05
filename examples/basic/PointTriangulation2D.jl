# a 2D triangulation example
# this example uses a factor graph to represent the data
# Factor graphs can be used to construct either fixed lag smoothing or full blown SLAM solutions.

# Load required libraries
using Caesar, Distributions
using RoMEPlotting


## Environment setup -- in Juno you can run cells with Alt+Enter

# true pose position (assume hidden)  - [X Y θ]
p0 = [1.0;1.0;0.0]

# true landmark positions
l0 = [0.0;1.0]
l1 = [1.0;0.0]
l2 = [2.0;1.0]


# noise level for Gaussian assumption -- see https://github.com/JuliaStats/Distributions.jl/issues/584
lcov = Matrix(Diagonal([0.01; 0.01].^2))

# measurements
m0 = pi/2
m1 = -pi
m2 = -pi/2


mstd = 0.05

## build the factor graph

fg = initfg()

# add two beacons
addVariable!(fg, :l0, Point2, tags=[:BEACON;])
addFactor!(fg, [:l0], Prior(MvNormal(l0, lcov)))

addVariable!(fg, :l1, Point2, tags=[:BEACON;])
addFactor!(fg, [:l1], Prior(MvNormal(l1, lcov)))

# addVariable!(fg, :l2, Point2, tags=[:BEACON;])
# addFactor!(fg, [:l2], Prior(MvNormal(l2, lcov)))


# add unknown pose location
addVariable!(fg, :x0, Pose2, tags=[:POSE;])

# add two bearing only measurements
addFactor!(fg, [:x0; :l0], Pose2Point2Bearing(Normal(m0, mstd)))
addFactor!(fg, [:x0; :l1], Pose2Point2Bearing(Normal(m1, mstd)))
# addFactor!(fg, [:x0; :l2], Pose2Point2Bearing(Normal(m2, mstd)))


# solve the system -- remember first runs are slow (just in time compile code)
[solveTree!(fg) for i in 1:5]


## Plot values to see result



plotPose(fg, :x0, axis=[-0.5 2.0; -0.5 1.5])


## more debugging


plotKDE(fg, [:l0; :l1],dims=[1;2],levels=3 ,axis=[0 2.0; 0.0 1.5])


stuff = localProduct(fg, :x0)


plotKDE(stuff[2],dims=[1;2],levels=3, c=["red","green","blue"])

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

getSolverParams(fg).drawpdf=true
getSolverParams(fg).showtree=true
tree = solveTree!(fg)


spyCliqMat(tree,:l0)

drawGraph(fg, show=true)




stuff = IIF.treeProductUp(fg, tree, :l0, :l0)

Gadfly.plot(x=stuff[1][1,:], y=stuff[1][2,:], Geom.histogram2d)



#
