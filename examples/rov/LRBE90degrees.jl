# visualize 90 degree yaw with ROV as bimodal example.

using Caesar
using RoME
using TransformUtils
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
gt[:x1] = [0.0;0.0;0;0;0;-pi/2]
gt[:x2] = [4.0;4.0;0;0;0;0]
gt[:l1] = [4.0;0.0;0.7]

# animate visualization
@async begin
  as = ArcPointsRangeSolve(
        [0.0;0;0.0],
        [-4.0/sqrt(2);4-4.0/sqrt(2);0.0],
        [-4.0;4.0;0.0], 4.0 )
  findaxiscenter!(as)
  animatearc(vc, rovt, as, initrot=Rotations.AngleAxis(pi/2,0,0,1.0))
end

# am = Translation(0,0,0.0) ∘ LinearMap(Rotations.Quat(1.0,0,0,0))
# rovt(vc, am )

# covariance parameters
initCov = 0.01*eye(6); [initCov[i,i] = 0.001 for i in 4:6];
odoCov, rangecov, bearingcov = deepcopy(initCov), 3e-4, 2e-4

# factor graph
fg = identitypose6fg(initpose=SE3(gt[:x1][1:3],Euler(gt[:x1][4:6]) ), initCov=initCov)
addOdoFG!(fg, Pose3Pose3(SE3(, Euler(gt[:x2][4:6]...) ), odoCov) )

visualizeallposes!(vc, fg, drawlandms=false)


addLinearArrayConstraint(fg, (4.12, 0.0), :x1, :l1, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (4.12, 0.0), :x1, :l2, rangecov=rangecov,bearingcov=bearingcov)

visualizeDensityMesh!(vc, fg, :l1, meshid=2)
















#
