
using Pkg
Pkg.activate(@__DIR__)


using DiffEqFlux, Flux, Optim, OrdinaryDiffEq

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



u0 = Float32(1.1)
tspan = (0.0f0,25.0f0)

ann = FastChain(FastDense(5,16,tanh), FastDense(16,16,tanh), FastDense(16,3))
p = initial_params(ann)
θ = Float32[u0;p]

function dudt_(u,p,t)
    ann([u;t;t],p)
end
prob = ODEProblem(dudt_,[0f0,0f0,u0],tspan,p)
concrete_solve(prob,Tsit5(),[0f0,0f0,u0],p,abstol=1e-8,reltol=1e-6)

function predict_adjoint(θ)
  Array(concrete_solve(prob,Tsit5(),[θ[1],0f0,0f0],θ[2:end],saveat=0.0:1:25.0))
end
loss_adjoint(θ) = sum(abs2,predict_adjoint(θ)[2,:].-1)
l = loss_adjoint(θ)

cb = function (θ,l)
  println(l)
  #display(plot(solve(remake(prob,p=Flux.data(p3),u0=Flux.data(u0)),Tsit5(),saveat=0.1),ylim=(0,6)))
  return false
end

# Display the ODE with the current parameter values.
cb(θ,l)

loss1 = loss_adjoint(θ)
res = DiffEqFlux.sciml_train(loss_adjoint, θ, BFGS(initial_stepnorm=0.01), cb = cb)
