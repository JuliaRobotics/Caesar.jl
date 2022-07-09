

using RoME
using Test
using GraphPlot

import Rotations as _Rot

##
@testset "Test ICRA Tutorial 1 CJL API code" begin
##

fg = initfg();

addVariable!(fg, :x0, Pose2);

prior_distribution = MvNormal(zeros(3), diagm([0.05,0.05,0.01].^2))
addFactor!(fg, [:x0], PriorPose2(prior_distribution)) 

DFG.plotDFG(fg);
@test !isInitialized(fg, :x0)

addVariable!(fg, :x1, Pose2);
odo_distribution = MvNormal([1.0, 0.0, pi/2], diagm([0.1, 0.1, 0.01].^2))
fac_1 = addFactor!(fg, [:x0, :x1], Pose2Pose2(odo_distribution))
DFG.plotDFG(fg);

solveGraph!(fg);

@test isapprox( [0;0;0],    getPPESuggested(fg, :x0); atol=0.2)
@test isapprox( [1;0;pi/2], getPPESuggested(fg, :x1); atol=0.2)

var = getVariable(fg, :x1)

@test isapprox( [1;0;pi/2], getPPESuggested(fg, :x1); atol=0.2)

addVariable!(fg, :l1, Point2)

p2br = Pose2Point2BearingRange(Normal(0.0,0.03),Normal(0.5,0.1))
addFactor!(fg, [:x0,:l1], p2br)

addVariable!(fg, :x2, Pose2)
addFactor!(fg, [:x1,:x2], Pose2Pose2(odo_distribution))

addVariable!(fg, :x3, Pose2)
addFactor!(fg, [:x2,:x3], Pose2Pose2(odo_distribution))

addVariable!(fg, :x4, Pose2)
addFactor!(fg, [:x3,:x4], Pose2Pose2(odo_distribution))

DFG.plotDFG(fg);

solveGraph!(fg);


@test isapprox( [0;0;0],    getPPESuggested(fg, :x0); atol=0.2)
@test isapprox( [1;0;pi/2], getPPESuggested(fg, :x1); atol=0.2)
@test isapprox( [0.5;0], getPPESuggested(fg, :l1); atol=0.1)

X2 = getBelief(fg, :x2)
x2 = mean(X2)
@test isapprox( ArrayPartition([1;1], _Rot.RotMatrix2(pi)), x2; atol=0.2)

X3 = getBelief(fg, :x3)
x3 = mean(X3)
@test isapprox( ArrayPartition([0;1], _Rot.RotMatrix2(-pi/2)), x3; atol=0.2)

X4 = getBelief(fg, :x4)
x4 = mean(X4)
@test isapprox( ArrayPartition([0;0], _Rot.RotMatrix2(0)), x4; atol=0.2)

X3 = getBelief(fg, :x3)
x3 = mean(X3)
@test isapprox( ArrayPartition([0;1], _Rot.RotMatrix2(-pi/2)), x3; atol=0.2)

# solve again with loop closure

p2br = Pose2Point2BearingRange(Normal(0.0,0.03),Normal(0.5,0.1))
addFactor!(fg, [:x4,:l1], p2br)

solveGraph!(fg)

@test isapprox( [0;0;0],    getPPESuggested(fg, :x0); atol=0.2)
@test isapprox( [1;0;pi/2], getPPESuggested(fg, :x1); atol=0.2)
@test isapprox( [0.5;0], getPPESuggested(fg, :l1); atol=0.1)

X2 = getBelief(fg, :x2)
x2 = mean(X2)
@test isapprox( ArrayPartition([1;1], _Rot.RotMatrix2(pi)), x2; atol=0.2)

X3 = getBelief(fg, :x3)
x3 = mean(X3)
@test isapprox( ArrayPartition([0;1], _Rot.RotMatrix2(-pi/2)), x3; atol=0.2)

X4 = getBelief(fg, :x4)
x4 = mean(X4)
@test isapprox( ArrayPartition([0;0], _Rot.RotMatrix2(0)), x4; atol=0.2)

X3 = getBelief(fg, :x3)
x3 = mean(X3)
@test isapprox( ArrayPartition([0;1], _Rot.RotMatrix2(-pi/2)), x3; atol=0.2)

@test getManifold(fg, :x1) isa AbstractManifold

##
end
##