
# using Revise
using RoME
using Manifolds
using Test

##


@testset "Test ICRA2022 Tutorial 4, multi-hypo and contradictory" begin
##


fg = initfg()

prior_distr= diagm([0.1, 0.1, 0.01].^2)
dual_distr= diagm([1, 1, sqrt(pi)].^2)


addVariable!(fg, :c0, Pose2)
addFactor!(fg, [:c0], PriorPose2(MvNormal([0.,0, 0], prior_distr)))

addVariable!(fg, :c1, Pose2)
addFactor!(fg, [:c1], PriorPose2(MvNormal([20.,0, pi/2], prior_distr)))

addVariable!(fg, :c2, Pose2)
addFactor!(fg, [:c2], PriorPose2(MvNormal([20.,10, pi], prior_distr)))

addVariable!(fg, :c3, Pose2)
addFactor!(fg, [:c3], PriorPose2(MvNormal([0.,10, -pi/2], prior_distr)))


addVariable!(fg, :door_prior, Pose2)
addFactor!(fg, [:door_prior], PriorPose2(MvNormal([20.,4, 0], prior_distr)))

addVariable!(fg, :door_dual, Pose2)
addFactor!(fg, [:door_dual], PriorPose2(MvNormal([20.,4, pi], dual_distr)))


solveGraph!(fg);


@test isapprox(getPPE(fg, :c0).suggested, [0;0;0]; atol=0.1)
@test isapprox(getPPE(fg, :c1).suggested, [20;0;pi/2]; atol=0.1)
@test isapprox(getPPE(fg, :c2).suggested .|> abs, [20;10;pi]; atol=0.1)
@test isapprox(getPPE(fg, :c3).suggested, [0;10;-pi/2]; atol=0.1)

@test isapprox(getPPE(fg, :door_prior).suggested, [20;4;0]; atol=0.1)
@test isapprox(getPPE(fg, :door_prior).suggested[1:2], [20;4]; atol=0.5)


##

addVariable!(fg, :x0, Pose2)
addFactor!(
  fg, 
  [:x0, :c0, :c1, :c2, :c3], 
  Pose2Pose2(MvNormal([-2, -2, 0.], diagm([0.5, 0.5, 0.05].^2))), 
  multihypo=[1.0, 0.25, 0.25, 0.25, 0.25]
)

solveGraph!(fg)

## test 4 modes

tp = zeros(3,1)

# near c0
tp[:,1] = [2;2;0.]
@test 0.15 < getBelief(fg, :x0)(tp)[1]
tp[:,1] = [2;2;pi/2]
@test getBelief(fg, :x0)(tp)[1] < 0.01

# near c1
tp[:,1] = [18;2;pi/2]
@test 0.15 < getBelief(fg, :x0)(tp)[1]
tp[:,1] = [18;2;0]
@test getBelief(fg, :x0)(tp)[1] < 0.01

# near c2
tp[:,1] = [18;8;-pi]
@test 0.15 < getBelief(fg, :x0)(tp)[1]
tp[:,1] = [18;8;0]
@test getBelief(fg, :x0)(tp)[1] < 0.01

# near c3
tp[:,1] = [2;8;-pi/2]
@test 0.15 < getBelief(fg, :x0)(tp)[1]
tp[:,1] = [2;8;0]
@test getBelief(fg, :x0)(tp)[1] < 0.01



##

addVariable!(fg, :x1, Pose2)
addFactor!(fg, [:x0, :x1], Pose2Pose2(MvNormal([4, 0, pi/2], diagm([0.5, 0.5, 0.05].^2))), nullhypo=0.5)

addFactor!(
  fg,
  [:x1, :c0, :c1, :c2, :c3], 
  Pose2Pose2(MvNormal([-2, -4, 0], diagm([0.5, 0.5, 0.05].^2))), 
  multihypo=[1.0, 0.25, 0.25, 0.25, 0.25]
)


##

solveGraph!(fg)

##


# near c1
tp[:,1] = [18;2;pi/2]
@test 0.15 < getBelief(fg, :x0)(tp)[1]
tp[:,1] = [18;2;0]
@test getBelief(fg, :x0)(tp)[1] < 0.01

# near c3
tp[:,1] = [2;8;-pi/2]
@test 0.15 < getBelief(fg, :x0)(tp)[1]
tp[:,1] = [2;8;0]
@test getBelief(fg, :x0)(tp)[1] < 0.01


# near c1
tp[:,1] = [18;6;-pi]
# weak test until KDE evaluation is on-manifold too
@test 0.02 < getBelief(fg, :x1)(tp)[1]
tp[:,1] = [18;6;pi] # FIXME, should be on-manifold
@test 0.02 < getBelief(fg, :x1)(tp)[1]
tp[:,1] = [18;6;0]
@test getBelief(fg, :x1)(tp)[1] < 0.01

# near c3
tp[:,1] = [2;4;0]
@test 0.15 < getBelief(fg, :x1)(tp)[1]
tp[:,1] = [2;4;pi/2]
@test getBelief(fg, :x1)(tp)[1] < 0.01


##

addVariable!(fg, :x2, Pose2)
addFactor!(fg, [:x1, :x2], Pose2Pose2(MvNormal([16, 0, 0], diagm([0.5, 0.5, 0.05].^2))))

addFactor!(fg, [:x2, :c0, :c1, :c2, :c3], Pose2Pose2(MvNormal([2, -4, pi/2], diagm([0.5, 0.5, 0.05].^2))), multihypo=[1.0, 0.25, 0.25, 0.25, 0.25])

solveGraph!(fg)


##

# easier testing for x2

M = Manifolds.SpecialEuclidean(2)
e0 = identity_element(M)

x2_a = MvNormal([18;4;0.],diagm([0.5;0.5;0.2.^2]))
pts2_a = [exp(M, e0, hat(M, e0, rand(x2_a))) for _ in 1:100]
X2_a = manikde!(M, pts2_a)

x2_b = MvNormal([2;6;-pi],diagm([0.5;0.5;0.2.^2]))
pts2_b = [exp(M, e0, hat(M, e0, rand(x2_b))) for _ in 1:100]
X2_b = manikde!(M, pts2_b)


##


# near c1
tp[:,1] = [18;2;pi/2]
@test 0.15 < getBelief(fg, :x0)(tp)[1]
tp[:,1] = [18;2;0]
@test getBelief(fg, :x0)(tp)[1] < 0.01

# near c3
tp[:,1] = [2;8;-pi/2]
@test 0.15 < getBelief(fg, :x0)(tp)[1]
tp[:,1] = [2;8;0]
@test getBelief(fg, :x0)(tp)[1] < 0.01


# near c1
tp[:,1] = [18;6;-pi]
# weak test until KDE evaluation is on-manifold too
@test 0.02 < getBelief(fg, :x1)(tp)[1]
tp[:,1] = [18;6;pi] # FIXME, should be on-manifold
@test 0.02 < getBelief(fg, :x1)(tp)[1]
tp[:,1] = [18;6;0]
@test getBelief(fg, :x1)(tp)[1] < 0.01

# near c3
tp[:,1] = [2;4;0]
@test 0.15 < getBelief(fg, :x1)(tp)[1]
tp[:,1] = [2;4;pi/2]
@test getBelief(fg, :x1)(tp)[1] < 0.01


@test mmd(X2_a, getBelief(fg, :x2) ) < 0.2
@test mmd(X2_b, getBelief(fg, :x2) ) < 0.2


##

addFactor!(fg, [:x2, :door_prior, :door_dual], Pose2Pose2(MvNormal([2.0, 0, pi], diagm([0.5, 0.5, 0.05].^2))), multihypo=[1,0.5,0.5])

solveGraph!(fg)

##


# near c3
tp[:,1] = [2;8;-pi/2]
@test 0.15 < getBelief(fg, :x0)(tp)[1]
tp[:,1] = [2;8;0]
@test getBelief(fg, :x0)(tp)[1] < 0.01


# near c3
tp[:,1] = [2;4;0]
@test 0.15 < getBelief(fg, :x1)(tp)[1]
tp[:,1] = [2;4;pi/2]
@test getBelief(fg, :x1)(tp)[1] < 0.01


# use synthetic belief to ensure x2 is correct
@test   mmd(X2_a, getBelief(fg, :x2)) < 0.2
@test !(mmd(X2_b, getBelief(fg, :x2)) < 0.2)

@test isapprox(    getPPE(fg, :door_prior).suggested[3], 0; atol=0.2) 
@test isapprox(abs(getPPE(fg, :door_dual ).suggested[3]), pi; atol=0.5)

##
end

#