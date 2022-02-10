## Contributed by @doublestrong

## Need 6 npz file and timing

using Caesar
# using RoME, Distributions
# using GraphPlot
using RoMEPlotting
Gadfly.set_default_plot_size(35cm,20cm)

# using Rebugger
# using Gadfly
using NPZ
using MAT

# # add multiproc support
using Distributed
addprocs(3)
using Caesar
@everywhere using Caesar, RoME

# # 90% precompile
for i in 1:5
  warmUpSolverJIT()
end


##

# # [OPTIONAL] add more julia processes to speed up inference
# using Distributed
# nprocs() < 3 ? addprocs(4-nprocs()) : nothing

# @everywhere using RoME
N = 1000
# start with an empty factor graph object
fg = initfg()

# fieldnames(typeof(getSolverParams(fg)))
# getSolverParams(fg).graphinit = true  # init on graph first then solve on tree (default)
# getSolverParams(fg).graphinit = false # init and solve on tree
getSolverParams(fg).useMsgLikelihoods = true
getSolverParams(fg).N = 150


# getSolverParams(fg).N = N
# # Add the first pose :x0
# addVariable!(fg, :x0, Pose2)
# # Add a few more poses
# for i in 1:5
#   addVariable!(fg, Symbol("x$(i)"), Pose2)
# end
# # Add landmarks
# for i in 1:2
#   addVariable!(fg, Symbol("l$(i)"), Point2)
# end



timing = zeros(0)


## Step 0

# Add at a fixed location Prior to pin :x0 to a starting location (0,0,0)
s_time = time_ns()
addVariable!(fg, :x0, Pose2)
addVariable!(fg, :l1, Point2)
addFactor!(fg, [:x0], PriorPose2( MvNormal([0; 0; 0], diagm([0.3;0.3;0.3].^2)) ))
ppr = Pose2Point2Range(MvNormal([7.323699045815659], diagm([0.3].^2)))
addFactor!(fg, [:x0; :l1], ppr )
# Gadfly.draw(PDF("fg.pdf", 10cm, 10cm),pl)  # or PNG(...)
# initAll!(fg)
# dfgplot(fg)

## Perform inference
tree = solveTree!(fg)
e_time = time_ns()
elapsed = (e_time - s_time)/10^9
append!(timing, elapsed)
println("elapsed time: ", elapsed)

# pl = plotKDE(fg, [:x0,:l1],dims=[1;2], levels=4)
pl = plotKDE(fg, [:x0,:l1],dims=[1;2], levels=4)


## 

Gadfly.draw(PDF("batch_1.pdf", 20cm, 10cm),pl)
X0 = rand(getBelief(fg, :x0) ,N)
L1 = rand(getBelief(fg, :l1) ,N)
batch1 = vcat(X0,L1)
npzwrite("batch1.npz",batch1)

# plX1 |> PNG("/tmp/testX1.png")
# X2 = rand(getBelief(fg, :x2) ,N)
# X3 = rand(getBelief(fg, :x3) ,N)
# X4 = rand(getBelief(fg, :x4) ,N)
# X5 = rand(getBelief(fg, :x5) ,N)
# res = vcat(X0,X1,L1)
# using NPZ
# npzwrite("res.npz",res)
# Gadfly.draw(PDF("batch.pdf", 10cm, 10cm),pl)  # or PNG(...)
# error("here's a breakpoint")

## Step 1
i = 1
s_time = time_ns()
addVariable!(fg, :x1, Pose2)
pp = Pose2Pose2(MvNormal([9.82039106655709;-0.040893643507463134;0.7851856362659602], diagm([0.3;0.3;0.05].^2)))
ppr = Pose2Point2Range(MvNormal([6.783355788017825], diagm([0.3].^2)))
addFactor!(fg, [Symbol("x$(i-1)"); Symbol("x$(i)")], pp )
addFactor!(fg, [Symbol("x$(i)"); :l1], ppr )


##

tree = solveTree!(fg, tree)

e_time = time_ns()
elapsed = (e_time - s_time)/10^9
append!(timing, elapsed)
println("elapsed time: ", elapsed)

pl = plotKDE(fg, [:x0,:x1,:l1],dims=[1;2], levels=4)

##

Gadfly.draw(PDF("batch_2.pdf", 20cm, 10cm),pl)
X0 = rand(getBelief(fg, :x0) ,N)
L1 = rand(getBelief(fg, :l1) ,N)
X1 = rand(getBelief(fg, :x1) ,N)
batch2 = vcat(X0,X1,L1)
npzwrite("batch2.npz",batch2)


# error("here's a breakpoint")
## Step 2

i = i + 1
s_time = time_ns()
addVariable!(fg, :x2, Pose2)
addVariable!(fg, :l2, Point2)
pp = Pose2Pose2(MvNormal([9.824301116341609;-0.1827443713759503;0.7586281983933238], diagm([0.3;0.3;0.05].^2)))
ppr = Pose2Point2Range(MvNormal([6.768864478545091], diagm([0.3].^2)))
addFactor!(fg, [Symbol("x$(i-1)"); Symbol("x$(i)")], pp )
addFactor!(fg, [Symbol("x$(i)"); :l2], ppr )


##


tree = solveTree!(fg, tree)
# tree = solveTree!(fg; smtasks, recordcliqs=ls(fg))

e_time = time_ns()
elapsed = (e_time - s_time)/10^9
append!(timing, elapsed)
println("elapsed time: ", elapsed)

pl = plotKDE(fg, [:x0,:x1,:x2,:l1,:l2],dims=[1;2], levels=4)

##
Gadfly.draw(PDF("batch_3.pdf", 20cm, 10cm),pl)
X0 = rand(getBelief(fg, :x0) ,N)
L1 = rand(getBelief(fg, :l1) ,N)
L2 = rand(getBelief(fg, :l2) ,N)
X1 = rand(getBelief(fg, :x1) ,N)
X2 = rand(getBelief(fg, :x2) ,N)
batch3 = vcat(X0,X1,X2,L1,L2)
npzwrite("batch3.npz",batch3)

#
## Step 3

i = i + 1
s_time = time_ns()
addVariable!(fg, :x3, Pose2)
pp = Pose2Pose2(MvNormal([9.776502885351334; -0.010587078502017132; 1.5591793408311467], diagm([0.3;0.3;0.05].^2)))
ppr = Pose2Point2Range(MvNormal([7.401417053438512], diagm([0.3].^2)))
addFactor!(fg, [Symbol("x$(i-1)"); Symbol("x$(i)")], pp )
addFactor!(fg, [Symbol("x$(i)"); :l2], ppr )

##
tree = solveTree!(fg, tree)
e_time = time_ns()
elapsed = (e_time - s_time)/10^9
append!(timing, elapsed)
println("elapsed time: ", elapsed)

pl = plotKDE(fg, [:x0,:x1,:x2,:x3,:l1,:l2],dims=[1;2], levels=4)

##
Gadfly.draw(PDF("batch_4.pdf", 20cm, 10cm),pl)
X0 = rand(getBelief(fg, :x0) ,N)
L1 = rand(getBelief(fg, :l1) ,N)
L2 = rand(getBelief(fg, :l2) ,N)
X1 = rand(getBelief(fg, :x1) ,N)
X2 = rand(getBelief(fg, :x2) ,N)
X3 = rand(getBelief(fg, :x3) ,N)
batch4 = vcat(X0,X1,X2,X3,L1,L2)
npzwrite("batch4.npz",batch4)

## Step 4 (:l2 has triple range measurements)
i = i + 1
s_time = time_ns()
addVariable!(fg, Symbol("x4"), Pose2)
pp = Pose2Pose2(MvNormal([9.644657137571507; 0.5847494762836476; 0.7422440549101994], Matrix(Diagonal([0.3;0.3;0.05].^2))))
ppr = Pose2Point2Range(MvNormal([7.331883435735532], Matrix(Diagonal([0.3].^2))))
addFactor!(fg, [Symbol("x$(i-1)"); Symbol("x$(i)")], pp )
addFactor!(fg, [Symbol("x$(i)"); Symbol("l2")], ppr )

##
tree = solveTree!(fg, tree)
e_time = time_ns()
elapsed = (e_time - s_time)/10^9
append!(timing, elapsed)
println("elapsed time: ", elapsed)

pl = plotKDE(fg, [:x0,:x1,:x2,:x3,:x4,:l1,:l2],dims=[1;2], levels=4)

##
Gadfly.draw(PDF("batch_5.pdf", 20cm, 10cm),pl)
X0 = rand(getBelief(fg, :x0) ,N)
L1 = rand(getBelief(fg, :l1) ,N)
L2 = rand(getBelief(fg, :l2) ,N)
X1 = rand(getBelief(fg, :x1),N)
X2 = rand(getBelief(fg, :x2) ,N)
X3 = rand(getBelief(fg, :x3) ,N)
X4 = rand(getBelief(fg, :x4),N)
batch5 = vcat(X0,X1,X2,X3,X4,L1,L2)
npzwrite("batch5.npz",batch5)

## Step 5 (triple range to :l1)
i = i + 1
s_time = time_ns()
addVariable!(fg, Symbol("x5"), Pose2)
pp = Pose2Pose2(MvNormal([9.725096752125593; 0.6094800276622434; 0.7750400402527422], Matrix(Diagonal([0.3;0.3;0.05].^2))))
ppr = Pose2Point2Range(MvNormal([6.476782344345081], Matrix(Diagonal([0.3].^2))))
addFactor!(fg, [Symbol("x$(i-1)"); Symbol("x$(i)")], pp )
addFactor!(fg, [Symbol("x$(i)"); Symbol("l1")], ppr )

##
tree = solveTree!(fg, tree)
e_time = time_ns()
elapsed = (e_time - s_time)/10^9
append!(timing, elapsed)
println("elapsed time: ", elapsed)

pl = plotKDE(fg, [:x0,:x1,:x2,:x3,:x4,:x5,:l1,:l2],dims=[1;2], levels=4)

##
Gadfly.draw(PDF("batch_6.pdf", 20cm, 10cm),pl)
X0 = rand(getBelief(fg, :x0),N)
L1 = rand(getBelief(fg, :l1),N)
L2 = rand(getBelief(fg, :l2),N)
X1 = rand(getBelief(fg, :x1),N)
X2 = rand(getBelief(fg, :x2),N)
X3 = rand(getBelief(fg, :x3),N)
X4 = rand(getBelief(fg, :x4),N)
X5 = rand(getBelief(fg, :x5),N)
batch6 = vcat(X0,X1,X2,X3,X4,X5,L1,L2)
npzwrite("batch6.npz",batch6)
npzwrite("timing.npz",timing)

#
#
# pl = plotSLAM2D(fg)
#
# # For scripting use-cases you can export the image
# Gadfly.draw(PDF("/tmp/test.pdf", 20cm, 10cm),pl)  # or PNG(...)
#
# L1 = rand(getBelief(fg, :l1) ,N)
# # L2 = rand(getBelief(fg, :l2) ,N)
# X0 = rand(getBelief(fg, :x0) ,N)
# X1 = rand(getBelief(fg, :x1) ,N)
# # X2 = rand(getBelief(fg, :x2) ,N)
# # X3 = rand(getBelief(fg, :x3) ,N)
# # X4 = rand(getBelief(fg, :x4) ,N)
# # X5 = rand(getBelief(fg, :x5) ,N)
# res = vcat(X0,X1,L1)
# using NPZ
# npzwrite("res.npz",res)
# # pl2 = Gadfly.plot(x=X0[1,:],y=X0[2,:], Geom.hexbin);
# # p1H = hstack(pl2)
# #
# # # convert to file
# # p1H |> PNG("/home/chad/Pictures/julia_x0_step0.png")
#
# # using RoMEPlotting
#
# #drawPoses(fg)
# # If you have landmarks, you can instead call
# # drawPosesLandms(fg)
# #
# # # # Draw the KDE for x0
# # # plotKDE(fg, :x0)
# # # # Draw the KDE's for x0 and x1
# # plotKDE(fg, [:x1])
# # plotKDE(fg, [:x2])
# # plotKDE(fg, [:x3])
# # plotKDE(fg, [:x4])
# # plotKDE(fg, [:x5])
# # plotKDE(fg, [:l1])
# # plotKDE(fg, [:l2])
