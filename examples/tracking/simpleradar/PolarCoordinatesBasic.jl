# target tracking -- cylindrical manifold (r, θ)

using Distributed

addprocs(3)

using Caesar
using DifferentialEquations
using ApproxManifoldProducts
using Caesar, RoME
using DataFrames
using TransformUtils


@everywhere using Caesar

@everywhere setForceEvalDirect!(true)


## Plotting libraries


using Colors
using Plots # for ODE
using KernelDensityEstimatePlotting, RoMEPlotting
using AbstractPlotting, Makie
using FileIO
using DelimitedFiles


## parameters for experiment

include(joinpath(dirname(@__FILE__),"GenerateSimData.jl"))


mc = 1; N = 100;

for mc = 1:50, N = [200;]

## Start building the factor graph in polar coordinates
include(joinpath(dirname(@__FILE__),"BuildFactorGraph.jl"))

##

# writeGraphPdf(fg, show=true)

# initAll!(fg)
getSolverParams(fg).drawtree=false
getSolverParams(fg).showtree=false
getSolverParams(fg).N=N
tree = solveTree!(fg)


# savejld(fg, file="fg100_rstd10_sol3.jld2")

##  Prep data for plotting

include(joinpath(dirname(@__FILE__),"PrepPlottingData.jl"))


## Calculate error statistics

mkdir( joinpath(dirname(@__FILE__),"exports","N$(N)_MC$(mc)") )
fname = joinpath(dirname(@__FILE__),"exports","N$(N)_MC$(mc)","meanRanErr.txt")
writedlm(fname, dferr["mean".==dferr[:,3],2], ',' )
fname = joinpath(dirname(@__FILE__),"exports","N$(N)_MC$(mc)","maxRanErr.txt")
writedlm(fname, dferr["max".==dferr[:,3],2], ',')
fname = joinpath(dirname(@__FILE__),"exports","N$(N)_MC$(mc)","meanAngErr.txt")
writedlm(fname, dferrang["mean".==dferrang[:,3],2], ',')
fname = joinpath(dirname(@__FILE__),"exports","N$(N)_MC$(mc)","maxAngErr.txt")
writedlm(fname, dferrang["max".==dferrang[:,3],2], ',')


end


## Generate all plots

include(joinpath(dirname(@__FILE__),"GeneratePlots.jl"))



## Look a factor graph local output -- More debugging

# pll, plc = plotLocalProductCylinder(fg, :t0, show=false, scale=0.1)








0


#
