# testing nearal ODE
# https://discourse.julialang.org/t/diffeqflux-with-time-as-additional-input-to-neural-ode/26456/3
# https://discourse.julialang.org/t/defining-and-input-signal-for-a-differential-equation/23703/10
# https://github.com/JuliaDiffEq/DiffEqFlux.jl#universal-differential-equations-for-neural-optimal-control
# https://docs.juliadiffeq.org/stable/tutorials/ode_example/#Defining-Parameterized-Functions-1

using Pkg
Pkg.activate(@__DIR__)
pkg"instantiate"
0
# pkg"precompile"


# using Revise

# using Flux
# using DifferentialEquations
# using DiffEqFlux, OrdinaryDiffEq, Optim


using DiffEqFlux, Flux, Optim, OrdinaryDiffEq


## Fix issue Zygote.jl #440 #516
using Zygote


Zygote.@adjoint function vcat(xs::Union{Number, AbstractVector}...)
    vcat(xs...), dy -> begin
        d = 0
        map(xs) do x
            x isa Number && return dy[d+=1]
            l = length(x)
            dx = dy[d+1:d+l]
            d += l
            return dx
        end
    end
end


## Load MAT VicPrk data

using TransformUtils
using Plots
using MAT

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
# plot(odo[:,1], odo[:,2])

ode_data = odo[START:STOP,:]' .|> Float32

u0 = Float32[0.; 0.; 0.]
datasize = size(ode_data,2)
tspan = (Float32(vpdata["time"][1]*1e-3), Float32(vpdata["time"][datasize]*1e-3))
ts = range(tspan[1],tspan[2],length=datasize)


## Specialized Neural model with time dependent inputs==========================

# model = Chain(x -> x.^3,
#               Dense(4,50,tanh),
#               Dense(50,3,tanh))
model = FastChain(FastDense(5,50,relu), FastDense(50,50,relu), FastDense(50,3,relu))
# model parameters
p = initial_params(model)

# regular ODE states -- i.e. non-time-dependent input states
u0 = zeros(Float32, 3) #Float32(1.1)
# θ is all non-time-dependent input parameters
θ = Float32[u0;p]

function dudt_(u,p,t)
  input = [u; t; t]
  model(input, p)
end


input0 = [0f0;0f0]
prob = ODEProblem(dudt_, u0, tspan, p)
concrete_solve(prob, Tsit5(), u0, p, abstol=1e-8, reltol=1e-6, saveat=ts)
Array(  concrete_solve(prob,Tsit5(), θ[1:3], θ[4:end], saveat=ts)  )


predict_adjoint(θ)


function predict_adjoint(θ)
  Array(  concrete_solve(prob,Tsit5(), θ[1:3], θ[4:end], saveat=0.0:1:25.0)  )
  # Array(concrete_solve(prob,Tsit5(),[0f0,0f0,θ[1]],θ[2:end],saveat=0.0:1:25.0))
end


function predict_rd()
# destructure param -> vector and back
  concrete_solve(prob_n_ode,Tsit5(),_u0,ps_m,saveat=ts)
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


## want to use sciml_train (Optim) -- Flux not as stable for stiff

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
