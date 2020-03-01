
# cd(@__DIR__)
using Pkg
Pkg.activate(@__DIR__)
pkg"instantiate"
0
pkg"precompile"


# using Revise

using Zygote
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





tspan = (0.0f0,25.0f0)

ann = FastChain(FastDense(5,16,tanh), FastDense(16,16,tanh), FastDense(16,3))
p = initial_params(ann)

# regular ODE states -- i.e. non-time-dependent input states
u0 = zeros(Float32, 3) #Float32(1.1)
# θ is all non-time-dependent input parameters
θ = Float32[u0;p]

function dudt_(u,p,t)
    ann([u;t;t],p)
end
input0 = [0f0;0f0]
prob = ODEProblem(dudt_, u0, tspan, p)
concrete_solve(prob, Tsit5(), u0, p, abstol=1e-8, reltol=1e-6, saveat=0.0:1:25.0)
Array(  concrete_solve(prob,Tsit5(), θ[1:3], θ[4:end], saveat=0.0:1:25.0)  )

function predict_adjoint(θ)
  Array(  concrete_solve(prob,Tsit5(), θ[1:3], θ[4:end], saveat=0.0:1:25.0)  )
  # Array(concrete_solve(prob,Tsit5(),[0f0,0f0,θ[1]],θ[2:end],saveat=0.0:1:25.0))
end

predict_adjoint(θ)

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





## Zygote issue 516


using Zygote
p = [1f0]
f(p) = [p;1f0][2]
Zygote.gradient(f,p)


ff(p) = [p;[1f0]][2]
Zygote.gradient(ff,p)


u0 = [1f0]
fff(p) = [p;u0][2]
Zygote.gradient(fff,p)


#
