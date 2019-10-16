using Caesar


fg = initfg()

addVariable!(fg, :x0, Pose2)
addVariable!(fg, :x1, Pose2)

addFactor!(fg, [:x0], PriorPose2(MvNormal(zeros(3),0.001*Matrix(I,3,3))))
addFactor!(fg, [:x1], PriorPose2(MvNormal([0;1;0.0],0.001*Matrix(I,3,3))))

addVariable!(fg, :l1, Point2)

addFactor!(fg, [:x0;:l1], Pose2Point2BearingRange(Normal(pi/4,0.01),Normal(1.0, 1.0)) )
addFactor!(fg, [:x1;:l1], Pose2Point2BearingRange(Normal(-pi/4,0.01),Normal(1.0, 1.0)) )

drawGraph(fg)

tree, smt, hist = solveTree!(fg)


using RoMEPlotting
Gadfly.set_default_plot_size(35cm, 25cm)

drawPosesLandms(fg, spscale=0.3)

plotKDE(fg, :l1, levels=3)

plotLocalProduct(fg, :l1, levels=2)

#
