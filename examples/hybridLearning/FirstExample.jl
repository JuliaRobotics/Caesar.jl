# testing nearal ODE

using DiffEqFlux, OrdinaryDiffEq, Flux, Optim, Plots

u0 = Float32[2.; 0.]
datasize = 30
tspan = (0.0f0,1.5f0)

function trueODEfunc(du,u,p,t)
    true_A = [-0.1 2.0; -2.0 -0.1]
    du .= ((u.^3)'true_A)'
end
t = range(tspan[1],tspan[2],length=datasize)
prob = ODEProblem(trueODEfunc,u0,tspan)
ode_data = Array(solve(prob,Tsit5(),saveat=t))

# dudt2 = FastChain((x,p) -> x.^3,
#             FastDense(2,50,tanh),
#             FastDense(50,2))
# n_ode = NeuralODE(dudt2,tspan,Tsit5(),saveat=t)

dudt2 = Chain(x -> x.^3,
             Dense(2,50,tanh),
             Dense(50,2))
n_ode = NeuralODE(dudt2,tspan,Tsit5(),saveat=t)


function predict_n_ode(p)
  n_ode(u0,p)
end

function loss_n_ode(p)
    pred = predict_n_ode(p)
    loss = sum(abs2,ode_data .- pred)
    loss,pred
end

loss_n_ode(n_ode.p) # n_ode.p stores the initial parameters of the neural ODE


cb = function (p,l,pred;doplot=false) #callback function to observe training
  display(l)
  # plot current prediction against data
  if doplot
    pl = scatter(t,ode_data[1,:],label="data")
    scatter!(pl,t,pred[1,:],label="prediction")
    display(plot(pl))
  end
  return false
end

# Display the ODE with the initial parameter values.
cb(n_ode.p,loss_n_ode(n_ode.p)..., doplot=true)

res1 = DiffEqFlux.sciml_train(loss_n_ode, n_ode.p, ADAM(0.05), cb = cb, maxiters = 300)
cb(res1.minimizer,loss_n_ode(res1.minimizer)...;doplot=true)
res2 = DiffEqFlux.sciml_train(loss_n_ode, res1.minimizer, LBFGS(), cb = cb)
cb(res2.minimizer,loss_n_ode(res2.minimizer)...;doplot=true)





## Try GPU


# u0 = Float32[2.; 0.] |> gpu
# dudt = Chain(Dense(2,50,tanh),Dense(50,2)) |> gpu
#
# # p,re = DiffEqFlux.destructure(dudt)
# p,re = Flux.destructure(dudt)
#
# dudt_(u,p,t) = re(p)(u)
# prob = ODEProblem(ODEfunc, u0,tspan, p)
#
# # Runs on a GPU
# sol = solve(prob,Tsit5(),saveat=0.1)
#
# n_ode = NeuralODE(gpu(dudt2),tspan,Tsit5(),saveat=0.1)

#
