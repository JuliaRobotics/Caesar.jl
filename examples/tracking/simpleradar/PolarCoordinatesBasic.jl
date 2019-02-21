# target tracking -- cylindrical manifold (r, θ)

using Caesar
using DifferentialEquations
using Plots # for ODE

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

u0 = [5,15,-10.0,50.0]
tspan = (0.0,5.0)
prob = ODEProblem(falling,u0,tspan)
sol = solve(prob)
# sol(t)

Plots.plot(sol, vars=(3,4))


## extract the desired measurement information from the physical simulation

nposes = 6
ts = range(0, stop=5.0, length=nposes)
XY = zeros(2,nposes)

for i in 1:nposes
  t = ts[i]
  XY[:,i] = sol(t)[3:4]
end

# double check
Plots.plot(XY[1,:], XY[2,:], linewidth=3)


## Calculate the true radar range and angle

ranges = sqrt.(vec(sum(XY.^2, dims=1)))
angles = atan.(XY[1,:]./XY[2,:])



# will look at only the first 5 values


## Start building the factor graph in polar coordinates

using Caesar

import IncrementalInference: getSample

struct Polar <: IIF.InferenceVariable
  dims::Int
  manifolds::Tuple{Symbol,Symbol}
  labels::Vector{String}
  Polar() = new(2,(:Euclid,:Circular),String[])
end

mutable struct PriorPolar{T1<:IIF.SamplableBelief, T2<:IIF.SamplableBelief} <: IIF.FunctorSingleton
  Zrange::T1
  Zangle::T2
  PriorPolar{T1,T2}() where {T1,T2} = new{T1,T2}()
  PriorPolar{T1,T2}(zr::T1, za::T2) where {T1 <: IIF.SamplableBelief,T2 <: IIF.SamplableBelief} = new{T1,T2}(zr,za)
end
PriorPolar(zr::T1, za::T2) where {T1 <: IIF.SamplableBelief,T2 <: IIF.SamplableBelief} = PriorPolar{T1,T2}(zr,za)

function getSample(pp2i::PriorPolar, N::Int=1)
  sps = zeros(2,N)
  sps[1,:] = rand(pp2i.Zrange,N);
  sps[2,:] = rand(pp2i.Zangle,N);
  return (sps, )
end


mutable struct PolarPolar{T1<:IIF.SamplableBelief, T2<:IIF.SamplableBelief} <: IIF.FunctorSingleton
  Zrange::T1
  Zangle::T2
  PolarPolar{T1,T2}() where {T1,T2} = new{T1,T2}()
  PolarPolar{T1,T2}(zr::T1, za::T2) where {T1 <: IIF.SamplableBelief,T2 <: IIF.SamplableBelief} = new{T1,T2}(zr,za)
end
PolarPolar(zr::T1, za::T2) where {T1 <: IIF.SamplableBelief,T2 <: IIF.SamplableBelief} = PolarPolar{T1,T2}(zr,za)

function getSample(pp2i::PolarPolar, N::Int=1)
  sps = zeros(2,N)
  sps[1,:] = rand(pp2i.Zrange,N);
  sps[2,:] = rand(pp2i.Zangle,N);
  return (sps, )
end

function (pp::PolarPolar)(res::Array{Float64},
                          userdata::FactorMetadata,
                          idx::Int,
                          meas::Tuple{Array{Float64,2}},
                          p1::Array{Float64,2},
                          p2::Array{Float64,2} )
  #
  @inbounds res[1:2] = meas[1][1:2,idx] - (p2[1:2,idx] - p1[1:2,idx])
  nothing
end

##

rstd = 3.0
astd = 0.1

fg = initfg()

addVariable!(fg, :t0, Polar)
addFactor!(fg, [:t0], PriorPolar(Normal(ranges[1],rstd), Normal(angles[1],astd)) )


addVariable!(fg, :t1, Polar)
addFactor!(fg, [:t1], PriorPolar(Normal(ranges[2],rstd), Normal(angles[2],astd)) )
addFactor!(fg, [:t0; :t1], PolarPolar(Normal(5.0, 10.0), Normal(0.0, 0.15)) )


addVariable!(fg, :t2, Polar)
addFactor!(fg, [:t2], PriorPolar(Normal(ranges[3],rstd), Normal(angles[3],astd)) )
addFactor!(fg, [:t1; :t2], PolarPolar(Normal(5.0, 10.0), Normal(0.0, 0.15)) )


addVariable!(fg, :t3, Polar)
addFactor!(fg, [:t3], PriorPolar(Normal(ranges[4],rstd), Normal(angles[4],astd)) )
addFactor!(fg, [:t2; :t3], PolarPolar(Normal(5.0, 10.0), Normal(0.0, 0.15)) )


addVariable!(fg, :t4, Polar)
addFactor!(fg, [:t4], PriorPolar(Normal(ranges[5],rstd), Normal(angles[5],astd)) )
addFactor!(fg, [:t3; :t4], PolarPolar(Normal(5.0, 10.0), Normal(0.0, 0.15)) )

writeGraphPdf(fg, show=true)


tree = batchSolve!(fg)


using ApproxManifoldProducts
using KernelDensityEstimatePlotting, RoMEPlotting

plotKDE(fg, [:t0;:t1;], levels=3)
plotKDE(fg, [:t2;:t3;:t4;], levels=3)



T0 = getKDE(fg, :t0)
T1 = getKDE(fg, :t1)
T2 = getKDE(fg, :t2)
T3 = getKDE(fg, :t3)
T4 = getKDE(fg, :t4)


T0a = marginal(T0, [2])
T1a = marginal(T1, [2])
T2a = marginal(T2, [2])
T3a = marginal(T3, [2])
T4a = marginal(T4, [2])


plotKDECircular([T0a;T1a;T2a;T3a;T4a])


m0 = getKDEMean(T0)
m1 = getKDEMean(T1)
m2 = getKDEMean(T2)
m3 = getKDEMean(T3)
m4 = getKDEMean(T4)


mm = [m0 m1 m2 m3 m4]

mt = [reshape(mm[1,:],5,1) reshape(ranges[1:5],5,1)]



Plots.plot(mt)
Plots.plot( ranges[1:5] )


stuff = localProduct(fg, :t2)

plotKDE([stuff[1];stuff[2]],levels=2,c=["black";"red";"blue";"green"])



im


##








#
