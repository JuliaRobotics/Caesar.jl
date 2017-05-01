# test range azimuth before using asfm data

using Caesar, RoME
using Distributions
using CoordinateTransformations


getRangeAziElevGT(gtl, posesym, pointsym) = convert(SE3, gtl[posesym]) \ Translation(gtl[pointsym][2]...)


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
gt[:l0] = (:XYZ, [3.0; -1.0; -00.5] )
gt[:l1] = (:XYZ, [1.0; 2.0; 0] )
gt[:l2] = (:XYZ, [3.0; 1.5; 0.5])


# XYZ qWqXqYqZ
gt[:x0]= (:XYZqWXYZ , [0.0; 0; 0;   1.0; 0; 0; 0] )
gt[:x1]= (:XYZqWXYZ , [1.0; 0; 0;   1.0; 0; 0; 0] )
gt[:x2]= (:XYZqWXYZ , [2.0; 0; 0;   1/sqrt(2); 0; 0; 1/sqrt(2)] )
gt[:x3]= (:XYZqWXYZ , [2.0; 0; 0.5;   1/sqrt(2); 0; 0; 1/sqrt(2)] )
q2 = convert(Quaternion, TransformUtils.AngleAxis(0.2,[1.0,0,0]))
q = Quaternion(1/sqrt(2), [0; 0; 1/sqrt(2)]) * q2
gt[:x4]= (:XYZqWXYZ , [2.0; 0; 0.5;   q.s; q.v...] )



wTx0 = convert(SE3, gt[:x0])
x0Tx1 = convert(SE3, gt[:x0]) \ convert(SE3, gt[:x1])
x1Tx2 = convert(SE3, gt[:x1]) \ convert(SE3, gt[:x2])
x2Tx3 = convert(SE3, gt[:x2]) \ convert(SE3, gt[:x3])
x3Tx4 = convert(SE3, gt[:x3]) \ convert(SE3, gt[:x4])


meas = Dict{Symbol, Tuple{Symbol,Vector{Float64}}}()
for x in 0:4, l in 0:2
  rae = getRangeAziElevGT(gt, Symbol("x$(x)"), Symbol("l$(l)") )
  meas[Symbol("x$(x)l$(l)")] = (:rangeazimuth, [rae.range, rae.azimuth])
end



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

addLinearArrayConstraint(fg, meas[:x0l2][2], :x0, :l2, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, meas[:x1l2][2], :x1, :l2, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, meas[:x2l2][2], :x2, :l2, rangecov=rangecov,bearingcov=bearingcov)


addOdoFG!(fg, Pose3Pose3( MvNormal(veePose3(x2Tx3), odoCov))  )

addLinearArrayConstraint(fg, meas[:x3l1][2], :x3, :l1, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, meas[:x3l2][2], :x3, :l2, rangecov=rangecov,bearingcov=bearingcov)



addOdoFG!(fg, Pose3Pose3( MvNormal(veePose3(x3Tx4), odoCov))  )

addLinearArrayConstraint(fg, meas[:x4l1][2], :x4, :l1, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, meas[:x4l2][2], :x4, :l2, rangecov=rangecov,bearingcov=bearingcov)



Graphs.plot(fg.g)
writeGraphPdf(fg)
# run(`evince fg.pdf`)
wipeBuildNewTree!(fg, drawpdf=true)
# run(`evince bt.pdf`)


visualizeallposes!(vc, fg, drawtype=:max, gt=gt)

[solveandvisualize(fg, vc, N=N, drawtype=:max, gt=gt) for i in 1:5];

visualizeDensityMesh!(vc, fg, :l0)
visualizeDensityMesh!(vc, fg, :l1)
visualizeDensityMesh!(vc, fg, :l2)

visualizeDensityMesh!(vc, fg, :x1)
visualizeDensityMesh!(vc, fg, :x3)

plotKDE(fg, :l2, dims=[1;3])







#
