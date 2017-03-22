addprocs(3)

using TransformUtils, Gadfly
using KernelDensityEstimate
using IncrementalInference
using RoME
using Caesar
# using Base.Test

# import RoME: ⊕

# Parametric solution with hand checked solution as ground truth
# XYZ
gt = Dict{Symbol, Tuple{Symbol,Vector{Float64}}}()
gt[:l1] =(:XYZ ,  [2.07618; -2.85304; 2.41059] )
gt[:l2] =(:XYZ ,  [2.27909; -3.53382; 1.5933] )
gt[:l3] =(:XYZ ,  [1.83487; -3.26179; 2.38189] )
gt[:l4] =(:XYZ ,  [1.85991; -3.26293; 2.77598] )
gt[:l5] =(:XYZ ,  [2.11136; -2.86976; 2.72259] )
gt[:l6] =(:XYZ ,  [1.79027; -3.22242; 2.15042] )
gt[:l7] =(:XYZ ,  [2.02859; -2.82187; 2.16534] )

# XYZ qWqXqYqZ
gt[:x1]= (:XYZqWXYZ , [0.000229458; 0.299991; -0.00230474;   0.696163;  0.0546845;  -0.123042;  0.705144] )
gt[:x2]= (:XYZqWXYZ , [-0.314997; -1.1901; 1.73293;          0.943023; -0.0847915;  -0.102545; -0.304964] )
gt[:x3]= (:XYZqWXYZ , [-0.363944; -1.18799; 1.74864;         0.937102; -0.0847783;  -0.108046; -0.320903] )
gt[:x4]= (:XYZqWXYZ , [-0.262489; -0.865555; 1.82557;        0.935268; -0.0797656; -0.0966424; -0.331015] )
gt[:x5]= (:XYZqWXYZ , [-0.680204; -1.37827; 1.73447;         0.949729; -0.0644836;  -0.120014; -0.281874] )
gt[:x6]= (:XYZqWXYZ , [-0.614131; -1.29263; 1.75964;         0.951792; -0.0638253;  -0.117314; -0.276146] )


# setup visualization process and default drawings
vc = startdefaultvisualization()


N = 200
fg = initfg()

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


println("Adding PriorPose3 to graph...")
init = SE3([0.0;0.0;0.0], Euler(-0.00772052, 0.0, -0.0992321))
v1 = addNode!(fg, :x1,  zeros(6,N) ⊕ Pose3Pose3(init, initCov),  N=N)
initPosePrior = PriorPose3(init, initCov)
f1  = addFactor!(fg,[v1], initPosePrior)


# Odometry pose 0 pose 1: (-0.380711, -1.02585, 1.77348; -2.19796, -0.151721, -0.0929671)
odo = SE3([-0.380711, -1.02585, 1.77348], Euler(-0.0929671, -0.151721, -2.19796) )
# v, f = addPose3Pose3(fg, "x2", deepcopy(odo), odoCov, N=N)
addOdoFG!(fg, Pose3Pose3(deepcopy(odo), odoCov) )

# Odometry pose 1 pose 2: (-0.00138117, -0.0600476, 0.0204065; -0.038278, 0.0186151, 0.00606331)
odo = SE3([-0.00138117; -0.0600476; 0.0204065], Euler(0.00606331, 0.0186151, -0.038278) )
# v, f = addPose3Pose3(fg, "x3", deepcopy(odo), odoCov, N=N)
addOdoFG!(fg, Pose3Pose3(deepcopy(odo), odoCov) )

# Odometry pose 2 pose 3: (-0.298704, -0.113974, 0.0244868; -0.00805163, -0.00425057, -0.0275746)
odo = SE3([-0.298704; -0.113974; 0.0244868], Euler(-0.0275746, -0.00425057, -0.00805163) )
# v, f = addPose3Pose3(fg, "x4", deepcopy(odo), odoCov, N=N)
addOdoFG!(fg, Pose3Pose3(deepcopy(odo), odoCov) )

# Odometry pose 3 pose 4: (0.670492, 0.0251882, -0.0705555; 0.109113, 0.0228966, 0.039246)
odo = SE3([0.670492; 0.0251882; -0.0705555], Euler(0.039246, 0.0228966, 0.109113) )
# v, f = addPose3Pose3(fg, "x5", deepcopy(odo), odoCov, N=N)
addOdoFG!(fg, Pose3Pose3(deepcopy(odo), odoCov) )

# Odometry pose 4 pose 5: (-0.0615969, 0.00849829, -0.0170747; 0.0315662, -0.0115416, -0.00605423)
odo = SE3([-0.0615969; 0.00849829; -0.0170747], Euler( -0.00605423,-0.0115416,0.0315662) )
# v, f = addPose3Pose3(fg, "x6", deepcopy(odo), odoCov, N=N)
addOdoFG!(fg, Pose3Pose3(deepcopy(odo), odoCov) )


println("Adding landmarks to graph...")
# Bearing Range pose 1 landmark 0: (0.0103329, 2.99139)
# Bearing Range pose 2 landmark 0: (0.0456434, 3.02145)
# Bearing Range pose 3 landmark 0: (-0.0277112, 3.12405)
# Bearing Range pose 4 landmark 0: (0.0778371, 3.20003)
# Bearing Range pose 5 landmark 0: (0.03676, 3.17675)

rangecov, bearingcov=3e-5, 3e-5

# vl1 = addNode!(fg, :l1,  0.1*randn(3,N),  N=N)
addLinearArrayConstraint(fg, (2.9914, 0.010333), :x2, :l1, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.02145, 0.04564), :x3, :l1, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.12405, -0.027711), :x4, :l1, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.2, 0.0778371), :x5, :l1, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.17675, 0.03676), :x6, :l1, rangecov=rangecov,bearingcov=bearingcov)

# Bearing Range pose 1 landmark 1: (-0.0971847, 3.49785)
# Bearing Range pose 2 landmark 1: (-0.0522769, 3.5344)
# Bearing Range pose 3 landmark 1: (-0.115672, 3.69064)
# Bearing Range pose 4 landmark 1: (-0.0471636, 3.66387)
# Bearing Range pose 5 landmark 1: (-0.0887547, 3.66507)
# addNode!(fg, :l2,  0.1*randn(3,N),  N=N)
addLinearArrayConstraint(fg, (3.49785, -0.0971847), :x2, :l2, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.5344, -0.0522769), :x3, :l2, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.69064, -0.115672), :x4, :l2, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.66387, -0.0471636), :x5, :l2, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.66507, -0.0887547), :x6, :l2, rangecov=rangecov,bearingcov=bearingcov)

# Bearing Range pose 1 landmark 2: (-0.149587, 3.06005)
# Bearing Range pose 2 landmark 2: (-0.0960879, 3.07682)
# Bearing Range pose 3 landmark 2: (-0.176641, 3.23283)
# Bearing Range pose 4 landmark 2: (-0.065336, 3.21081)
# Bearing Range pose 5 landmark 2: (-0.115751, 3.20317)
# addNode!(fg, :l3,  0.1*randn(3,N),  N=N)
addLinearArrayConstraint(fg, (3.06005, -0.149587), :x2, :l3, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.07682, -0.0960879), :x3, :l3, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.23283, -0.176641), :x4, :l3, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.21081, -0.065336), :x5, :l3, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.20317, -0.115751), :x6, :l3, rangecov=rangecov,bearingcov=bearingcov)



# Bearing Range pose 1 landmark 3: (-0.151731, 3.18027)
# Bearing Range pose 2 landmark 3: (-0.099368, 3.20953)
# Bearing Range pose 3 landmark 3: (-0.174175, 3.33787)
# Bearing Range pose 4 landmark 3: (-0.0662606, 3.32899)
# Bearing Range pose 5 landmark 3: (-0.114886, 3.32137)
# addNode!(fg, :l4,  0.1*randn(3,N),  N=N)
addLinearArrayConstraint(fg, (3.18027, -0.151731),:x2, :l4, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.20953, -0.099368), :x3, :l4, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.33787, -0.174175),:x4, :l4, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.32899, -0.0662606),:x5, :l4, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.32137, -0.114886),:x6, :l4, rangecov=rangecov,bearingcov=bearingcov)

# Bearing Range pose 1 landmark 4: (0.00322788, 3.10878)
# Bearing Range pose 2 landmark 4: (0.0370399, 3.15066)
# Bearing Range pose 3 landmark 4: (-0.030071, 3.22893)
# Bearing Range pose 4 landmark 4: (0.0683279, 3.31492)
# Bearing Range pose 5 landmark 4: (0.0323075, 3.29366)
# addNode!(fg, :l5,  0.1*randn(3,N),  N=N)
addLinearArrayConstraint(fg, (3.10878, 0.00322788),:x2, :l5, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.15066, 0.0370399), :x3, :l5, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.22893, -0.030071), :x4, :l5, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.31492, 0.0683279), :x5, :l5, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.29366, 0.0323075), :x6, :l5, rangecov=rangecov,bearingcov=bearingcov)

# Bearing Range pose 1 landmark 5: (-0.146491, 2.95272)
# Bearing Range pose 2 landmark 5: (-0.090717, 2.99605)
# Bearing Range pose 3 landmark 5: (-0.178344, 3.14063)
# Bearing Range pose 4 landmark 5: (-0.0603386, 3.10463)
# Bearing Range pose 5 landmark 5: (-0.111392, 3.10912)
# addNode!(fg, :l6,  0.1*randn(3,N),  N=N)
addLinearArrayConstraint(fg, (2.95272, -0.146491), :x2, :l6, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (2.99605, -0.090717), :x3, :l6, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.14063, -0.178344), :x4, :l6, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.10463, -0.0603386), :x5, :l6, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.10912, -0.111392), :x6, :l6, rangecov=rangecov,bearingcov=bearingcov)

# Bearing Range pose 1 landmark 6: (0.0179481, 2.8873)
# Bearing Range pose 2 landmark 6: (0.0543992, 2.9317)
# Bearing Range pose 3 landmark 6: (-0.0250711, 3.03224)
# Bearing Range pose 4 landmark 6: (0.0878667, 3.09855)
# Bearing Range pose 5 landmark 6: (0.041406, 3.07297)
# addNode!(fg, :l7,  0.1*randn(3,N),  N=N)
addLinearArrayConstraint(fg, (2.8873, 0.0179481), :x2, :l7, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (2.9317, 0.0543992), :x3, :l7, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.03224, -0.0250711), :x4, :l7, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.09855, 0.0878667), :x5, :l7, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (3.07297, 0.041406), :x6, :l7, rangecov=rangecov,bearingcov=bearingcov)


# solving = [true]
# @async begin
#   while solving[1]
#     println("visualizing")
#     visualizeallposes!(vc, fg)
#     visualizeDensityMesh!(vc, fg, :l1)
#     sleep(2)
#   end
# end
#
# tree = wipeBuildNewTree!(fg,drawpdf=false);
# @time inferOverTree!(fg, tree)
# solving[1]=false;

solveandvisualize(fg, vc, N=N, drawtype=:fit)




visualizeallposes!(vc, fg, drawtype=:fit)


# ls(fg)

visualizeDensityMesh!(vc, fg, :l6)


[inferOverTree!(fg, tree) for i in 1:3]; # should not be required


draw(PDF("/home/dehann/Desktop/test.pdf",30cm,20cm),
 plotKDE( getVertKDE(fg,:l1), dimLbls=["x";"y";"z";"phi";"the";"psi"]) )


 draw(PDF("/home/dehann/Desktop/test.pdf",30cm,20cm),
  plotKDE( pl1, dimLbls=["x";"y";"z";"phi";"the";"psi"]) )


draw(PDF("/home/dehann/Desktop/test.pdf",30cm,20cm),
 plotKDE( getVertKDE(fg,:x6), dimLbls=["x";"y";"z";"phi";"the";"psi"]) )

XX = String["x$i" for i in 1:6];
LL = String["l$i" for i in 1:7];
[draw(PDF("/home/dehann/Desktop/imgs/$(i).pdf",30cm,20cm),
  plotKDE( getVertKDE(fg,i), dimLbls=["x";"y";"z";"phi";"the";"psi"]) ) for i in union(XX,LL)];




# # new install
# Pkg.add("KernelDensityEstimate")
#
# # or update
# Pkg.update()
#
# # draw this
# using KernelDensityEstimate, Gadfly
#
# p, q = kde!(randn(2,100)), kde!(randn(2,100)+2);
#
# plotKDE(p)
# plotKDE([p;q],c=["red";"blue"],levels=3)
#
# p, q = kde!(randn(4,100)), kde!(randn(4,100)+2);
# draw(PDF("test.pdf",30cm,20cm),
#         plotKDE( [p;q],c=["red";"blue"],levels=3,
# 				dimLbls=["x";"y";"z";"phi";"the";"psi"]) )









#
