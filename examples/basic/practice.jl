# readme example


#addprocs(2)
using Caesar, RoME, TransformUtils, Distributions

# load scene and ROV model (might experience UDP packet loss LCM buffer not set)
vc = startdefaultvisualization()
sc1 = loadmodel(:scene01); sc1(vc)
rovt = loadmodel(:rov); rovt(vc)


initCov = 0.001*eye(6); [initCov[i,i] = 0.00001 for i in 4:6];
odoCov = 0.0001*eye(6); [odoCov[i,i] = 0.00001 for i in 4:6];
rangecov, bearingcov = 3e-4, 2e-3

# start and add to a factor graph
fg = identitypose6fg(initCov=initCov)
tf1 = SE3([0.0;0.0;0.0], Euler(0,0.0,0.0) )
tf2 = SE3([0.0;0.0;0.0], Euler(pi/2,0.0,0.0) )
tf3 = SE3([0.0;0.0;0.0], Euler(-pi/2,0.0,0.0) )

tf4 = SE3([0.0;5.0;0.0], Euler(0,0.0,0.0) )
tf5 = SE3([4.0;4.0;0.0], Euler(0,0.0,-pi/2) )

tf6 =  SE3([0.0;5.0;0.0], Euler(0,0.0,0.0) )
tf7 = SE3([4.0;4.0;0.0], Euler(0,0.0,-pi/2) )

tf8 =  SE3([0.0;5.0;0.0], Euler(0,0.0,0.0) )
tf9 = SE3([4.0;4.0;0.0], Euler(0,0.0,-pi/2) )

addOdoFG!(fg, Pose3Pose3(MvNormal(veeEuler(tf1), odoCov) ) )
addOdoFG!(fg, Pose3Pose3(MvNormal(veeEuler(tf2), odoCov) ) )
addOdoFG!(fg, Pose3Pose3(MvNormal(veeEuler(tf3), odoCov) ) )


addOdoFG!(fg, Pose3Pose3(MvNormal(veeEuler(tf4), odoCov) ) )
addOdoFG!(fg, Pose3Pose3(MvNormal(veeEuler(tf5), odoCov) ) )

addOdoFG!(fg, Pose3Pose3(MvNormal(veeEuler(tf6), odoCov) ) )
addOdoFG!(fg, Pose3Pose3(MvNormal(veeEuler(tf7), odoCov) ) )

addOdoFG!(fg, Pose3Pose3(MvNormal(veeEuler(tf8), odoCov) ) )
addOdoFG!(fg, Pose3Pose3(MvNormal(veeEuler(tf9), odoCov) ) )

visualizeallposes!(vc, fg, drawlandms=false)

# addLinearArrayConstraint(fg, (4.0, 0.0), :x0, :l1, rangecov=rangecov,bearingcov=bearingcov)
# visualizeDensityMesh!(vc, fg, :l1)
# addLinearArrayConstraint(fg, (4.0, 0.0), :x1, :l1, rangecov=rangecov,bearingcov=bearingcov)
# addLinearArrayConstraint(fg, (4.0, 0.0), :x2, :l1, rangecov=rangecov,bearingcov=bearingcov)
# visualizeDensityMesh!(vc, fg, :l1)


addLinearArrayConstraint(fg, (4.0, 0.0), :x4, :l2, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (4.0, 0.0), :x5, :l2, rangecov=rangecov,bearingcov=bearingcov)

addLinearArrayConstraint(fg, (4.0, 0.0), :x6, :l3, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (4.0, 0.0), :x7, :l3, rangecov=rangecov,bearingcov=bearingcov)

addLinearArrayConstraint(fg, (4.0, 0.0), :x8, :l4, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (4.0, 0.0), :x9, :l4, rangecov=rangecov,bearingcov=bearingcov)

batchSolve(fg)

# visualize(fg, vc, drawlandms=true, densitymeshes=[:l1;:x2])
visualize(fg, vc, drawlandms=true, densitymeshes=[:l2;:x4])
visualize(fg, vc, drawlandms=true, densitymeshes=[:l3;:x7])
visualize(fg, vc, drawlandms=true, densitymeshes=[:l4;:x9])

Graphs.plot(fg.g)




#
