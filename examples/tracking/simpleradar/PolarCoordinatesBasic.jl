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



## resolve the ground truth trajectory from ordinary differential equations

# ddy = g
# dx/dt = Fx + Gu
# [ddy;   = [0  0][dy  + [1][g=-10]
#   dy ]    [1  0]  y]   [0]
function falling(du,u,p,t)
  du[1] = 0    # drag
  du[2] = -10  # gravity
  du[3] = u[1]
  du[4] = u[2]
  nothing
end

u0 = [5,15,-20.0,0.0]
tspan = (0.0,5.0)
prob = ODEProblem(falling,u0,tspan)
sol = solve(prob)
# sol(t)

Plots.plot(sol, vars=(3,4))


## extract the desired measurement information from the physical simulation

tstop = 5.0
nposes = 10
ts = range(0, stop=tstop, length=nposes)
XY = zeros(2,nposes)

for i in 1:nposes
  t = ts[i]
  XY[:,i] = sol(t)[3:4]
end

gx = (t)->sol(t)[3]
gy = (t)->sol(t)[4]





# Calculate the true radar range and angle

ranges = sqrt.(vec(sum(XY.^2, dims=1)))
angles = atan.(XY[2,:], XY[1,:])


XYdense = [gx.(0:0.01:tstop) gy.(0:0.01:tstop)]';
randense = sqrt.(vec(sum(XYdense.^2, dims=1)))
angdense = atan.(XYdense[2,:], XYdense[1,:])


pl

## Start building the factor graph in polar coordinates


rstd = 3.0
astd = 0.1

rrm = 5.0
rrstd = 20.0
aam = 0.0
aastd = 0.2

N = 100
fg = initfg()

addVariable!(fg, :t0, Polar)
addFactor!(fg, [:t0], PriorPolar(Normal(ranges[1],rstd), Normal(angles[1],astd)) )

for tidx in 1:(nposes-1)
  psym = Symbol("t$(tidx-1)")
  sym = Symbol("t$tidx")
  addVariable!(fg, sym, Polar)
  addFactor!(fg, [sym], PriorPolar(Normal(ranges[tidx+1],rstd), Normal(angles[tidx+1],astd)) )
  addFactor!(fg, [psym; sym], PolarPolar(Normal(rrm, rrstd), Normal(aam, aastd)) )
end


writeGraphPdf(fg, show=true)

##

# ensureAllInitialized!(fg)

tree = batchSolve!(fg)

##

# pts0 = [2pi*rand(1,n).-pi;]
# setValKDE!(fg, :t0, manikde!(randn()))


##  Prep data for plotting

include(joinpath(dirname(@__FILE__),"PrepPlottingData.jl"))

## Generate all plots

##

batchSolve!(fg)



## Look a factor graph local output


plotLocalProduct(fg, :t0, show=false)


plotLocalProduct(fg, :t1, show=false)


plotLocalProduct(fg, :t2, show=false)


##

writeGraphPdf(fg, show=true)

##


## Need to build testing for Polar




#
