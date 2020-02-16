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
# using DiffEqFlux, OrdinaryDiffEq, Optim


using DiffEqFlux, Flux, Optim, OrdinaryDiffEq
using DifferentialEquations

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
using Interpolations

vpdata = matread(ENV["HOME"]*"/data/dataset_victoria_park/original_MATLAB_dataset/aa3_dr.mat")


# utility functions

include(joinpath(@__DIR__, "..", "wheeled", "parametricWheelOdo.jl"))

vpdata["time"] .*= 1e-3
vpdata["time"] = round.(vpdata["time"][:], digits=5)


vpsensors = hcat(vpdata["time"],
                 vpdata["speed"],
                 vpdata["steering"])
#

steer =  LinearInterpolation( (vpdata["time"],) , vpdata["steering"][:])
speed =  LinearInterpolation( (vpdata["time"],) , vpdata["speed"][:])



odo = allOdoEasy(vpsensors)
# vcat(ode_data, zeros(Float32, size()))

START=1
STOP=5000
plot(odo[START:STOP,1], odo[START:STOP,2])
# plot(odo[:,1], odo[:,2])

ode_data = odo[START:STOP,:]' .|> Float32

# x_i, y_i, theta_i, vx_i
u0 = Float32[0.; 0.; 0.; 0.]
# u0 = zeros(Float32, 4) #Float32(1.1)

datasize = size(ode_data,2)
tspan = (Float32(vpdata["time"][1]), Float32(vpdata["time"][datasize]))
ts = range(tspan[1],tspan[2],length=datasize)


## Specialized Neural model with time dependent inputs==========================

# model = Chain(x -> x.^3,
#               Dense(4,50,tanh),
#               Dense(50,3,tanh))
model = FastChain(FastDense(6,20,relu), FastDense(20,20,relu), FastDense(20,3,relu))
# model parameters
p = initial_params(model)

# regular ODE states -- i.e. non-time-dependent input states
# θ is all non-time-dependent input parameters
θ = Float32[u0;p]

t0 = vpdata["time"][1] |> Float32



function dudt_(u,p,t)
  # sp = 0 < t ? speed(t) : 0.0
  # st = 0 < t ? steer(t) : 0.0
  # tidx = findmin(abs.(vpdata["time"][:]*1e-3 .- t))[2]
  tidx = round(Int, (t-t0)*40+1)
  tidx = 1 < tidx ? tidx : 1
  sp = vpdata["speed"][tidx, 1] + 1e-8*randn()
  st = vpdata["steering"][tidx, 1]

  # @show t, sp, st # cant differentiate
  input = [u; sp; st]
  model(input, p)
end


# count = 0
# incrementCount(x...) = global count += 1
# periodic = PeriodicCallback(incrementCount, 0.25)


input0 = [0f0;0f0]
prob = ODEProblem(dudt_, u0, tspan, p)
concrete_solve(prob, Tsit5(), u0, p, abstol=1e-8, reltol=1e-6, saveat=ts)
Array(  concrete_solve(prob,Tsit5(), θ[1:4], θ[5:end], saveat=ts) )# , callback=periodic  )


function predict_adjoint(θ)
  Array(  concrete_solve(prob, Tsit5(), θ[1:4], θ[5:end], saveat=ts)  )
end

predict_adjoint(θ)




function loss_adjoint(θ)
  pred = predict_adjoint(θ)
  loss = 0.0
  for idx in 1:size(ode_data,2)
    loss = (ode_data[1,idx]-pred[1,idx])^2 + (ode_data[2,idx]-pred[2,idx])^2 + (cos(ode_data[2,idx])-cos(pred[2,idx]))^2 + (sin(ode_data[2,idx])-sin(pred[2,idx]))^2
  end
  loss
end

l = loss_adjoint(θ)

# Callback
cb = function (θ,l)
  println(l)
  #display(plot(solve(remake(prob,p=Flux.data(p3),u0=Flux.data(u0)),Tsit5(),saveat=0.1),ylim=(0,6)))
  return false
end

cb(θ,l)


# saved_values = SavedValues(Float64, Tuple{Float64,Float64})
# cb = SavingCallback((u,t,integrator)->(tr(u),norm(u)), saved_values)


loss1 = loss_adjoint(θ)

res = DiffEqFlux.sciml_train(loss_adjoint, θ, ADAM(0.1), cb = cb, maxiters=1000)
# res = DiffEqFlux.sciml_train(loss_adjoint, θ, BFGS(initial_stepnorm=0.01), cb = cb)


# import Tracker: param
# import Zygote: param, Params
# import Flux: param
# #
# param(x::Params) = x

#





#
