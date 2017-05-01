# test range azimuth before using asfm data

using Caesar, RoME
using Distributions
using CoordinateTransformations

# start a visualizer
vc = startdefaultvisualization();


# noise setup
sqrtinv = [[10;0;0;0;0;0]';
[0;10;0;0;0;0]';
[0;0;10;0;0;0]';
[0;0;0;57.2958;0;0]';
[0;0;0;0;57.2958;0]';
[0;0;0;0;0;57.2958]'];

initCov = inv(sqrtinv^2)
odoCov = deepcopy(initCov)

sqrtinv = [[286.479;0]';
[0;200]']
brCov = inv(sqrtinv^2)

rangecov, bearingcov=3e-5, 3e-5



# preload ground truth information
gt = Dict{Symbol, Tuple{Symbol,Vector{Float64}}}()
gt[:l0] =(:XYZ ,  [3.0; -1.0; 0] )
gt[:l1] =(:XYZ ,  [1.0; 2.0; 0] )


# XYZ qWqXqYqZ
gt[:x0]= (:XYZqWXYZ , [0.0; 0; 0;   1.0; 0; 0; 0] )
gt[:x1]= (:XYZqWXYZ , [1.0; 0; 0;   1.0; 0; 0; 0] )
gt[:x2]= (:XYZqWXYZ , [2.0; 0; 0;   1/sqrt(2); 0; 0; 1/sqrt(2)] )


wTx0 = convert(SE3, gt[:x0])
x0Tx1 = convert(SE3, gt[:x0]) \ convert(SE3, gt[:x1])
x1Tx2 = convert(SE3, gt[:x1]) \ convert(SE3, gt[:x2])


getRangeAziElevGT(gt, posesym, pointsym) = convert(SE3, gt[posesym]) \ Translation(gt[pointsym][2]...)

meas = Dict{Symbol, Tuple{Symbol,Vector{Float64}}}()
for x in 0:2, l in 0:1
  rae = getRangeAziElevGT(gt, Symbol("x$(x)"), Symbol("l$(l)") )
  meas[Symbol("x$(x)l$(l)")] = (:rangeazimuth, [rae.range, rae.azimuth])
end


meas[:x0l0]
meas[:x1l0]

meas[:x2l1]


# actual graph operations
N = 100
fg = initfg()

println("Adding PriorPose3 to graph...")
v0 = addNode!(fg, :x0,  zeros(6,N) ⊕ Pose3Pose3(MvNormal(veePose3(wTx0),initCov) ),  N=N)
initPosePrior = PriorPose3(MvNormal(veePose3(wTx0), initCov))
f1  = addFactor!(fg,[v0], initPosePrior)

addOdoFG!(fg, Pose3Pose3( MvNormal(veePose3(x0Tx1), odoCov))  )
addOdoFG!(fg, Pose3Pose3( MvNormal(veePose3(x1Tx2), odoCov))  )


addLinearArrayConstraint(fg, meas[:x0l0][2], :x0, :l0, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, meas[:x1l0][2], :x1, :l0, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, meas[:x2l1][2], :x2, :l1, rangecov=rangecov,bearingcov=bearingcov)

Graphs.plot(fg.g)




visualizeallposes!(vc, fg, drawtype=:fit, gt=gt)

solveandvisualize(fg, vc, N=N, drawtype=:fit, gt=gt)













#
