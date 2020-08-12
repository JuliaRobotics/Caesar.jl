# https://diffeqflux.sciml.ai/dev/examples/LV-Univ/

using DiffEqFlux, Flux, Optim, OrdinaryDiffEq, Plots
using DataInterpolations

u0 = 1.1f0
tspan = (0.0f0, 25.0f0)
tsteps = 0.0f0:1.0:25.0f0

#
noise_itp = LinearInterpolation( 0.001*randn(Float32, length(tsteps)), tsteps )

model_univ = FastChain(FastDense(2, 16, tanh),
                       FastDense(16, 16, tanh),
                       FastDense(16, 1))

# The model weights are destructured into a vector of parameters
p_model = initial_params(model_univ)
n_weights = length(p_model)

# Parameters of the second equation (linear dynamics)
p_system = Float32[0.5, -0.5]

p_all = [p_model; p_system]


function dudt_univ!(du, u, p, t)
    # Destructure the parameters
    model_weights = p[1:n_weights]
    α = p[end - 1]
    β = p[end]

    # The neural network outputs a control taken by the system
    # The system then produces an output
    model_control, system_output = u

    # adding some noise to the system output
    system_output += noise_itp(t)

    # Dynamics of the control and system
    dmodel_control = model_univ(u, model_weights)[1]
    dsystem_output = α*system_output + β*model_control

    # Update in place
    du[1] = dmodel_control
    du[2] = dsystem_output
end

prob_univ = ODEProblem(dudt_univ!, u0, tspan, p_all)
sol_univ = concrete_solve(prob_univ, Tsit5(),[0f0, u0], p_all,
                          abstol = 1e-8, reltol = 1e-6)

function predict_univ(θ)
  return Array(concrete_solve(prob_univ, Tsit5(), [0f0, θ[1]], θ[2:end],
                              saveat = tsteps))
end

loss_univ(θ) = sum(abs2, predict_univ(θ)[2,:] .- 1)

θ = Float32[u0; p_all]
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
                                     BFGS(initial_stepnorm = 0.01),
                                     cb = callback)
#

#
