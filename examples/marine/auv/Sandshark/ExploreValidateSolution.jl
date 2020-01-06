# validate a previous solve

# using Distributed
# addprocs(8)

using Revise

using DistributedFactorGraphs
using Caesar, RoME
# @everywhere using Caesar, RoME
using LCMCore, BotCoreLCMTypes
using Dates
using DataStructures
using Logging
using JSON2
using DSP

# few local application specific functions
include(joinpath(@__DIR__, "SandsharkUtils.jl"))
include(joinpath(@__DIR__, "Plotting.jl"))


logpath = "/media/dehann/temp2/caesar/2020-01-05T13:22:16.359/"

fg = initfg()
loadDFG(logpath*"fg_final", Main, fg)


dontMarginalizeVariablesAll!(fg)
plotLocalProduct(fg, :x533)

Juno.@enter plotLocalProduct(fg, :x533)


ls(fg, :x533)


# # draw plots in getLogPath(fg)
# include(joinpath(@__DIR__, "GenerateResults.jl"))


## BATCH SOLVE

# dontMarginalizeVariablesAll!(fg)
# tree, smt, hist = solveTree!(fg)


#
