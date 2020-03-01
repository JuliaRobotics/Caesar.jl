# build a factor graph from PS3 eye acoustic data


# Load required libraries
using Caesar, RoMEPlotting
# using Distributions
# using ApproxManifoldProducts

## Environment setup -- in Juno you can run cells with Alt+Enter

# true pose position (assume hidden)  - [X Y θ]
p0 = [0.0;0.0;0.0]

# noise level for Gaussian assumption -- see https://github.com/JuliaStats/Distributions.jl/issues/584
pcov = Matrix(Diagonal([0.01; 0.01; 0.05].^2))
lcov = Matrix(Diagonal([0.01; 0.01].^2))


# true landmark positions
l0 = [0.4841;0.559/2]
l1 = [0.4841;-0.559/2]

# d0 = l0-p0[1:2]
# m0 = atan(d0[2],d0[1])
#
# d1 = l1-p0[1:2]
# m1 = atan(d1[2],d1[1])



# mstd = 0.1

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
addFactor!(fg, [:x0; :l0], Pose2Point2BearingRange(pcLeft, Rayleigh(2.0)))
addFactor!(fg, [:x0; :l1], Pose2Point2BearingRange(pcRight, Rayleigh(2.0)))

writeGraphPdf(fg, show=true)

##

solveTree!(fg)

##

drawPosesLandms(fg, spscale=0.5)


getKDEMean(getKDE(fg, :l0))
getKDEMax(getKDE(fg, :l0))


getKDEMean(getKDE(fg, :l1))
getKDEMax(getKDE(fg, :l1))



## OPTION 2




fg = initfg()

# add two beacons
addVariable!(fg, :l0, Point2, labels=["BEACON"])
addFactor!(fg, [:l0], Prior(MvNormal(l0, lcov)))

addVariable!(fg, :l1, Point2, labels=["BEACON"])
addFactor!(fg, [:l1], Prior(MvNormal(l1, lcov)))


# add unknown pose location
addVariable!(fg, :x0, Pose2, labels=["POSE"])
addFactor!(fg, [:x0], PartialPrior(Normal(0.0, 0.01), (2,)) )
# addFactor!(fg, [:x0], PartialPrior(Uniform(-1.0, 0.5), (1,)) )
# addFactor!(fg, [:x0], PriorPose2(MvNormal(p0, pcov)))

# add two bearing only measurements
addFactor!(fg, [:x0; :l0], Pose2Point2BearingRange(pcLeft, Rayleigh(1.0)))
addFactor!(fg, [:x0; :l1], Pose2Point2BearingRange(pcRight, Rayleigh(1.0)))

writeGraphPdf(fg, show=true)

##

getSolveParams(fg).N = 100
solveTree!(fg)


## look at one mode only

pts = getVal(fg, :x0)
pts0 = pts[:,pts[1,:] .< 1.0]
X00 = kde!(pts0[1,:])
@show getKDEMax(X00)[1]
plotKDE(X00)

##


pl = plotPose(Pose2(), [getKDE(fg, :x0);])

drawPosesLandms(fg, spscale=0.5, meanmax=:mean, xmin=-3.0, xmax=3.0)


getKDEMean(getKDE(fg, :x0))
getKDEMax(getKDE(fg, :x0))


getKDEMean(getKDE(fg, :l0))
getKDEMax(getKDE(fg, :l0))


getKDEMean(getKDE(fg, :l1))
getKDEMax(getKDE(fg, :l1))


##


pl |> PDF("/tmp/test.pdf",30cm,20cm)
pl |> SVG("/tmp/test.svg",15cm,6cm)

##

@async run(`evince /tmp/test.pdf`)

##
