# https://diffeqflux.sciml.ai/dev/examples/LV-Univ/

using DiffEqFlux, Flux, Optim, OrdinaryDiffEq, Plots
using DataInterpolations

using JLD2
using Dates


t100 = DateTime("2018-07-25T22:23:51.39")
t101 = DateTime("2018-07-25T22:23:51.721")
dt = (t101-t100).value*1e-3 |> Float32
JLD2.@load joinpath(@__DIR__, "oneOdo.jld2") jvd X100 X101


vx0, vy0 = jvd[1,3], jvd[1,4] # 0, 0
x0 = [0.0;0;vx0;vy0]
tspan = (0.0f0, dt)
tsteps = range(tspan[1],tspan[2],length=size(jvd,1))  # 0.0f0:1.0:25.0f0

#
throttle_itp = LinearInterpolation( jvd[:,1], tsteps )
sterring_itp = LinearInterpolation( jvd[:,2], tsteps )

# take in x and u
model_g = FastChain(FastDense( 6, 16, tanh),
                    FastDense(16, 16, tanh),
                    FastDense(16, 4) )
#

# The model weights are destructured into a vector of parameters
p_model = initial_params(model_g)
n_weights = length(p_model)

# # Parameters of the second equation (linear dynamics)
# p_system = Float32[0.0; 0.0, vx0, vy0]

p_all = p_model # [p_model; p_system]
θ = Float32[p_all; x0]

# dx/dt = Fx + g(x,u)
function dxdt_odo!(dx, x, p, t)
  # Destructure the parameters
  @assert length(p) == n_weights "dxdt_odo! model weight lengths do not match"
  # model_weights = p[1:n_weights]
  # state = p[(end-3):end]

  # Getting the throttle and steering inputs (from interpolators)
  thr = throttle_itp(t)
  str = sterring_itp(t)

  # Update in place
  # Velocity model dynamics and TBD odometry model
  # The neural network outputs odometry from joystick values
  dx[1:4] .= model_g([x;thr;str], p)
  dx[1:2] .+= x[3:4]
  dx
end


prob_odo = ODEProblem(dxdt_odo!, x0, tspan, p_all)
sol_odo = concrete_solve(prob_odo, Tsit5(), x0, p_all,
                          abstol = 1e-8, reltol = 1e-6)

function predict_univ(θ)
  # solving for both the ODE solution and network parameters
  x0_ = θ[(end-3):end]
  p_ = θ[1:(end-4)]
  return Array(concrete_solve(prob_odo, Tsit5(), x0_, p_,
                              saveat = tsteps))
end

function loss_univ(θ)
  pred = predict_univ(θ)
  return sum(abs2, (X101[1:2,1] - X100[1:2,1]) - pred[1:2,end]) +
         sum(abs2, pred[3:4,1] .- pred[3:4,:])
end
l = loss_univ(θ)


list_plots = []
iter = 0
callback = function (θ, l)
  global list_plots, iter

  if iter == 0
    list_plots = []
  end
  iter += 1

  println(l)

  plt = Plots.plot(predict_univ(θ), ylim = (0, 6))
  push!(list_plots, plt)
  display(plt)
  return false
end


result_univ = DiffEqFlux.sciml_train(loss_univ, θ,
                                     ADAM(0.01),
                                     cb = callback,
                                     maxiters=1000)
                                     # BFGS(initial_stepnorm = 0.01),
#

#
