# target tracking -- cylindrical manifold (r, θ)

using Caesar
using DifferentialEquations
using ApproxManifoldProducts
using Caesar, RoME
using DataFrames

import IncrementalInference: getSample

setForceEvalDirect!(true)


## Plotting libraries


using Colors
using Plots # for ODE
using KernelDensityEstimatePlotting, RoMEPlotting
using AbstractPlotting, Makie
using FileIO


## parameters for experiment


include(joinpath(dirname(@__FILE__),"GenerateSimData.jl"))


## Start building the factor graph in polar coordinates

include(joinpath(dirname(@__FILE__),"BuildFactorGraph.jl"))


##

writeGraphPdf(fg, show=true)

# ensureAllInitialized!(fg)
tree = batchSolve!(fg, drawpdf=true, show=true)



##  Prep data for plotting

include(joinpath(dirname(@__FILE__),"PrepPlottingData.jl"))

## Generate all plots

include(joinpath(dirname(@__FILE__),"GeneratePlots.jl"))




## Look a factor graph local output -- More debugging

# pll, plc = plotLocalProductCylinder(fg, :t0, show=false, scale=0.1)








0


#
