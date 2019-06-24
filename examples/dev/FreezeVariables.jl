using Caesar

robotId = "Munster"
sessionId = "Test"

fg = initfg(sessionname=sessionId, robotname=robotId)

# also add a PriorPose2 to pin the first pose at a fixed location
addVariable!(fg, :x0, Pose2) # , labels=["POSE"]
addFactor!(fg, [:x0], PriorPose2(MvNormal(zeros(3), 0.01*Matrix(LinearAlgebra.I, 3,3))) )


# Drive around in a hexagon
for i in 0:5
  psym = Symbol("x$i")
  nsym = Symbol("x$(i+1)")
  addVariable!(fg, nsym, Pose2) # , labels=["VARIABLE";"POSE"]
  addFactor!(fg, [psym;nsym], Pose2Pose2( MvNormal([10.0;0;pi/3], 0.01*Matrix(LinearAlgebra.I, 3,3)) ), autoinit=true )
  # Pose2Pose2_NEW(MvNormal([10.0;0;pi/3], diagm([0.1;0.1;0.1].^2)))
end


response = addVariable!(fg, :l1, Point2("LANDMARK"))
