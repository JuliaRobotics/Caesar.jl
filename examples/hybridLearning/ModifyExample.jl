# testing nearal ODE
# https://discourse.julialang.org/t/diffeqflux-with-time-as-additional-input-to-neural-ode/26456/3
# https://discourse.julialang.org/t/defining-and-input-signal-for-a-differential-equation/23703/10
# https://github.com/JuliaDiffEq/DiffEqFlux.jl#universal-differential-equations-for-neural-optimal-control
# https://docs.juliadiffeq.org/stable/tutorials/ode_example/#Defining-Parameterized-Functions-1

cd(@__DIR__)
using Pkg
pkg"activate ."


# using Revise

using Flux
using DifferentialEquations
using DiffEqFlux, OrdinaryDiffEq, Optim, Plots
using Zygote, DiffEqSensitivity
using MAT
using TransformUtils

## Load MAT VicPrk data


vpdata = matread(ENV["HOME"]*"/data/dataset_victoria_park/original_MATLAB_dataset/aa3_dr.mat")


# utility functions

include(joinpath(@__DIR__, "..", "wheeled", "parametricWheelOdo.jl"))

vpsensors = hcat(vpdata["time"]*1e-3,
                 vpdata["speed"],
                 vpdata["steering"])
#

odo = allOdoEasy(vpsensors)
# vcat(ode_data, zeros(Float32, size()))

START=1
STOP=5000
plot(odo[START:STOP,1], odo[START:STOP,2])

ode_data = odo[START:STOP,:]' .|> Float32

u0 = Float32[0.; 0.; 0.]
datasize = size(ode_data,2)
tspan = (Float32(vpdata["time"][1]*1e-3), Float32(vpdata["time"][datasize]*1e-3))
ts = range(tspan[1],tspan[2],length=datasize)


## Specialized Neural model with time dependent inputs==========================

# model = Chain(x -> x.^3,
#               Dense(4,50,tanh),
#               Dense(50,3,tanh))
model = Chain(Dense(5,50,tanh),
              Dense(50,3,tanh))
#

function dudt_(u,p,t)
  input = [u; t; t]
  # Flux.Tracker.collect(model(input))
  model(input)
end

ps_m = Flux.params(model)

# p = Float32[0.0]
# p = param(p)
_u0 = u0
# _u0 = param(u0)


prob_n_ode = ODEProblem(dudt_, u0, tspan, ps_m)


# diffeq_rd(p, prob_n_ode, Tsit5())  # Test run
concrete_solve(prob_n_ode,Tsit5(),_u0,ps_m,saveat=ts,sensealg=TrackerAdjoint())
concrete_solve(prob_n_ode,MethodOfSteps(Tsit5()),_u0,ps_m,saveat=ts,sensealg=TrackerAdjoint())
concrete_solve(prob_n_ode,Tsit5(),_u0,ps_m,saveat=ts,sensealg=ForwardDiffSensitivity())
concrete_solve(prob_n_ode,Tsit5(),_u0,ps_m,saveat=ts)



function predict_rd()
  concrete_solve(prob_n_ode,MethodOfSteps(Tsit5()),_u0,ps_m,saveat=ts,sensealg=TrackerAdjoint())
  # concrete_solve(prob_n_ode,Tsit5(),_u0,ps_m,saveat=ts,sensealg=TrackerAdjoint())
  # concrete_solve(prob_n_ode,Tsit5(),_u0,ps_m,saveat=ts,sensealg=ZygoteAdjoint())
  # concrete_solve(prob_n_ode, Tsit5(), _u0, ps_m, saveat=ts, sensealg=ForwardDiffSensitivity())
  # concrete_solve(prob_n_ode,Tsit5(),_u0,ps_m,saveat=ts)
  # diffeq_rd(p, prob_n_ode, Tsit5(), saveat=ts, u0=_u0) # deprecated
end


predict_rd()


function loss_n_ode2()
  pred = predict_rd()
  # loss = sum(abs2,ode_data .- pred)
  # Q = [1 0 0; 0 1 0; 0 0 1]
  loss = 0.0
  # res = zeros(3)
  for idx in 1:size(ode_data,2)
  #   jXjhat = SE2(ode_data[1:3,idx]) \ SE2(pred[1:3,idx])
  #   se2vee!(res, jXjhat)
    # loss += res'*Q*res
    loss = (ode_data[1,idx]-pred[1,idx])^2 + (ode_data[2,idx]-pred[2,idx])^2 + (cos(ode_data[2,idx])-cos(pred[2,idx]))^2 + (sin(ode_data[2,idx])-sin(pred[2,idx]))^2
  end
  loss #,pred
end

ps_m = Flux.params(model)

loss_n_ode2()

# Callback
cb = function()
    display(loss_n_ode2())
end



fxdata = Iterators.repeated((), 30)
opt = ADAM()
cb()  # Test call


Flux.train!(loss_n_ode2, ps_m, fxdata, opt, cb=Flux.throttle(cb, 1))


0

# res = DiffEqFlux.sciml_train(loss_n_ode2, _u0, ADAM(),maxiters=30)
# res1 = DiffEqFlux.sciml_train(loss_n_ode2, ps_m, ADAM(0.05), maxiters = 300)




# import Tracker: param
# import Zygote: param, Params
# import Flux: param
# #
# param(x::Params) = x


#
