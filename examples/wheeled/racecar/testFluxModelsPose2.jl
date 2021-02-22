# test FluxModelsPose2Pose2

using LinearAlgebra
using Flux
using RoME

import RoME: FluxModelsPose2Pose2

##

include(joinpath((@__DIR__), "LoadPyNNTxt.jl"))

allModels = []
for i in 0:99
# /home/dehann/data/racecar/results/conductor/models/retrained_network_weights0
  push!(allModels, loadTfModelIntoFlux(ENV["HOME"]*"/data/racecar/results/conductor/models/retrained_network_weights$i") )
end

##

# start with a basic factor graph
fg = generateCanonicalFG_ZeroPose2()

mvnNaive = MvNormal(zeros(3), diagm([1.0;1.0;0.01]))

addVariable!(fg, :x1, Pose2)

jvd = zeros(25,4)
pp = FluxModelsPose2Pose2(allModels, jvd, mvnNaive, 0.5)

##

addFactor!(fg, [:x0;:x1], pp)


pts = approxConv(fg, :x0x1f1, :x1)

#




##  Special async task to add Neural odo to fg when data becomes available.

# # latest pose
# allVars = ls(fec.slam.dfg, r"x\d") |> sortDFG
# # does it have FluxModelsPose2Pose2 factor?
# allFluxFct = ls(fec.slam.dfg, FluxModelsPose2Pose2)
# mask = (x->intersect(ls(fec.slam.dfg, x), allFluxFct ) |> length).(allVars) .== 0
# varsWithout = allVars[mask][end-5:end]
#
# # what are the command values from previous pose
# let prevPs = prevPs, i=i
# prevPs = varsWithout[1]
# i = 2
# for i in 2:length(varsWithout)
#
# ps = varsWithout[i]
# theVar = getVariable(fec.slam.dfg, ps)
# # skip if no entry yet
# # hasDataEntry(theVar, :JOYSTICK_CMD_VALS) ? nothing : continue
# cmdData = fetchDataElement(theVar, fec.datastore, :JOYSTICK_CMD_VALS)
# throttle = (x->x[3].axis[2]).(cmdData)
# steering = (x->x[3].axis[4]).(cmdData)
# DT = ( getTimestamp(theVar) - getTimestamp(getVariable(fec.slam.dfg, prevPs)) ).value*1e-3
# xj = getPPE(fec.slam.dfg, ps).suggested
# xi = getPPE(fec.slam.dfg, prevPs).suggested
# biRw = TU.R(-xi[3])
# biV = biRw * (xj[1:2] - xi[1:2]) / DT
# biVrep = repeat(biV', length(throttle))
# joyval = hcat(throttle, steering, biVrep)
#
# #
# @show prevPs, ps, size(joyval,1)
#
# joyval
# itpJoy = interpToOutx4(joyval)
#
# JlOdoPredictorPoint2(itpJoy,allModels)
#
#
# prevPs = ps
# end
# end
#
# # interpolate joyvals to right size
# # joyval
#
# # the naive model (should be the camera)
# DXmvn = MvNormal(zeros(3),diagm([0.4;0.1;0.4].^2))
#
# fmp2 = FluxModelsPose2Pose2(x->JlOdoPredictorPoint2(x,allModels), joyval25, DXmvn,0.6)
#
#
# # convert to [25 x 4] input
