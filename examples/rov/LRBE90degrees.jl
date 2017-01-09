# visualize 90 degree yaw with ROV as bimodal example.

using Caesar
using RoME
using TransformUtils
using IncrementalInference
# using Rotations
# using CoordinateTransformations


# start visualizer with some scene
vc = startdefaultvisualization(draworigin=false)
sleep(3.0)
defaultscene01!(vc)

# load necessary data
rovt = loadmodel(:rov)
rovt(vc)

# ground truth positions
gt = Dict{Symbol, Vector{Float64}}()
gt[:x1] = [0.0;0.0;0;0;0;pi/2]
gt[:x2] = [-4.0;4.0;0;0;0;0]
gt[:l1] = [0.0;4.0;0.7]


# am = Translation(0,0,0.0) ∘ LinearMap(Rotations.Quat(1.0,0,0,0))
# rovt(vc, am )

# covariance parameters
initCov = 0.05*eye(6); [initCov[i,i] = 0.001 for i in 4:6];
odoCov = 0.001*eye(6); [odoCov[i,i] = 0.001 for i in 4:6];
rangecov, bearingcov = 3e-2, 2e-2
tfx1 = SE3(gt[:x1][1:3],Euler(gt[:x1][4:6]) )
tfx2 = SE3(gt[:x2][1:3],Euler(gt[:x2][4:6]) )

# factor graph
N=200
fg = identitypose6fg(initpose=tfx1, initCov=initCov, N=N)

addOdoFG!(fg, Pose3Pose3(SE3([4.0;4;0],Euler(0,0,-pi/2)), odoCov) )

visualizeallposes!(vc, fg, drawlandms=false)
addLinearArrayConstraint(fg, (4.23, 0.0), :x1, :l1, rangecov=rangecov,bearingcov=bearingcov)
visualizeDensityMesh!(vc, fg, :l1, meshid=2)


# animate visualization
# @async begin
  as = ArcPointsRangeSolve(
        [0.0;0;0.0],
        [-4.0/sqrt(2);4-4.0/sqrt(2);0.0],
        [-4.0;4.0;0.0], 4.0 )
  findaxiscenter!(as)
  sleep(5)
  animatearc(vc, rovt, as, initrot=Rotations.AngleAxis(pi/2,0,0,1.0), delaytime=0.02)
  sleep(5)
# end


addLinearArrayConstraint(fg, (4.23, 0.0), :x2, :l1, rangecov=rangecov,bearingcov=bearingcov)
visualizeallposes!(vc, fg, drawlandms=false)

pose2prior = PriorPose3(tfx2, initCov)
addFactor!(fg,[getVert(fg, :x2)], pose2prior)

solveandvisualize(fg, vc, densitymeshes=[:l1], N=N)















#
