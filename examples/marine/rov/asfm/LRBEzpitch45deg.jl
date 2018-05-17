# pitch and z case

using Caesar
using RoME
using TransformUtils

# modify ROV init orientation
using CoordinateTransformations
using Rotations

# start visualizer with some scene
vc = startdefaultvisualization(draworigin=true)
sleep(3.0)

# load necessary data
rovt = loadmodel(:rov)
rovt(vc)

am = Translation(0,0,0.0) ∘ LinearMap(Rotations.AngleAxis(pi/2,0,0,1.0))
rovt(vc, am )

# defaultscene01!(vc)
sc01 = loadmodel(:scene01)
sc01(vc)

# ground truth positions
gt = Dict{Symbol, Vector{Float64}}()
gt[:x1] = [0.0;0.0;0;0;0;pi/2]
gt[:x2] = [0.0;4.0-4.0/sqrt(2);4.0/sqrt(2);0;0;pi/2]
gt[:l1] = [0.0;4.0;0.7]

# prepare for animation
as = ArcPointsRangeSolve(
[0.0;0;0.0],
[0.0;4.0-4.0/sqrt(2);4.0/sqrt(2)],
[0.0;4.0;4.0], 4.0 )
findaxiscenter!(as)

# covariance parameters
initCov = 0.01*eye(6); [initCov[i,i] = 0.001 for i in 4:6];
odoCov = 0.001*eye(6); [odoCov[i,i] = 0.001 for i in 4:6];
rangecov, bearingcov = 3e-4, 2e-3
tfx1 = SE3(gt[:x1][1:3],Euler(gt[:x1][4:6]) )

sleep(3)
# factor graph
N=300
fg = identitypose6fg(initpose=tfx1, initCov=initCov, N=N)

visualizeallposes!(vc, fg, drawlandms=false)

rho1 = norm(tfx1.t-gt[:l1])
addLinearArrayConstraint(fg, (rho1, 0.0), :x1, :l1, rangecov=rangecov,bearingcov=bearingcov)

visualizeDensityMesh!(vc, fg, :l1)


# animate visualization
# @async begin

initrot=Rotations.AngleAxis(pi/2,0,0,1.0)
animatearc(vc, rovt, as, initrot=initrot, delaytime=0.02,to=0.5)
# end

am = parameterizeArcAffineMap(0.5, as, initrot=initrot )
am = LinearMap(Rotations.AngleAxis(-pi/2,0,0,1.0)) ∘ am
px2 = SE3(am.v[1:3] ,Quaternion(am.m.w, [am.m.x;am.m.y;am.m.z] ))
@show px2.t

addOdoFG!(fg, Pose3Pose3(MvNormal(veeEuler(px2), odoCov) ) )

@show rho2 = norm((tfx1*px2).t-gt[:l1])
addLinearArrayConstraint(fg, (rho2, 0.0), :x2, :l1, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (rho2, 0.0), :x2, :l2, rangecov=rangecov,bearingcov=bearingcov)
visualizeDensityMesh!(vc, fg, :l2)
sleep(3)


batchSolve(fg)
visualize(fg, vc, densitymeshes=[:l1, :l2], N=N, drawlandms=false)

# should just be the point where landmark likelihoods sightings intersect
deletemeshes!(vc)
visualizeDensityMesh!(vc, fg, :l1)









#
