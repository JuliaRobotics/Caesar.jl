# testing nearal ODE
# https://discourse.julialang.org/t/diffeqflux-with-time-as-additional-input-to-neural-ode/26456/3
# https://discourse.julialang.org/t/defining-and-input-signal-for-a-differential-equation/23703/10
# https://github.com/JuliaDiffEq/DiffEqFlux.jl#universal-differential-equations-for-neural-optimal-control
# https://docs.juliadiffeq.org/stable/tutorials/ode_example/#Defining-Parameterized-Functions-1

using Pkg
Pkg.activate(@__DIR__)
pkg"instantiate"
0
pkg"precompile"

using ArgParse

function parse_commandline()
    s = ArgParseSettings()

    @add_arg_table! s begin
        "--individuals"
            help = "Number of simultaneous training individuals to compute"
            arg_type = Int
            default = 1
        "--maxiters"
            help = "Max ADAM optimization iterations"
            arg_type = Int
            default = 100
        "--epochs"
            help = "number of encoder readings (epochs) to consume for pose to pose trigger"
            arg_type = Int
            default = 40
        "--segments"
            help = "number of interpose segments to combine"
            arg_type = Int
            default = 20
        # "--flag1"
        #     help = "an option without argument, i.e. a flag"
        #     action = :store_true
        # "arg1"
        #     help = "a positional argument"
        #     required = true
    end

    return parse_args(s)
end

pargs = parse_commandline()


# using Revise

# using Flux
# using DiffEqFlux, OrdinaryDiffEq, Optim


using Dates, DelimitedFiles
using DiffEqFlux, Flux, Optim, OrdinaryDiffEq
using DifferentialEquations


# Requires Julia 1.3 and up
import Base.Threads.@spawn

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

# see the input data
# plot(vpsensors[:,2])
# plot(vpsensors[:,3])

# # Likely unused
# steer =  LinearInterpolation( (vpdata["time"],) , vpdata["steering"][:])
# speed =  LinearInterpolation( (vpdata["time"],) , vpdata["speed"][:])


## Get baseline solution over all data
odo = allOdoEasy(vpsensors)
# vcat(ode_data, zeros(Float32, size()))

# calculate local speed
deltadist = sum( (odo[1:end-1,1:2] - odo[2:end,1:2]).^2, dims=2) .|> sqrt |> vec
push!(deltadist, deltadist[end])
bodySpeed = deltadist*40

numEpochs = pargs["epochs"]

U0s = zeros(5,pargs["segments"])
#second and beyond
for i in 1:pargs["segments"]
  @show epoch = (i-1)*numEpochs + 1
  U0s[:,i] = [odo[epoch,1];odo[epoch,2];cos(odo[epoch,3]);sin(odo[epoch,3]);bodySpeed[epoch]]
end

U0s


# Callback to show current cost during training
cb = function (θ,l)
  println("thread=$(Threads.threadid()), cost=$l")
  return false
end



# Train a single instancwe
function doTraining(model::FastChain,
                    u0_all::Matrix{<:Real};
                    p::Vector{<:Real}=initial_params(model),
                    maxiters=50,
                    α=0.05/(10*rand()) )
  #
  global pargs, tspan

  function dudt_(u,p,t)
    tidx = round(Int, (t-tspan[1])*40+1) # interpolation approach has issues with train
    tidx = 1 < tidx ? tidx : 1
    sp = vpdata["speed"][tidx, 1] + 1e-6*randn()
    st = vpdata["steering"][tidx, 1]
    input = [u; sp; st]
    model(input, p)
  end

  tspans = []
  # tspan1 = tspan
  # tspan2 = (tspan[1]+1.0, tspan[2]+1)
  probs = []
  for i in 1:pargs["segments"]
    push!(tspans, (tspan[1].+(i-1), tspan[2].+(i-1)))
    push!(probs, ODEProblem(dudt_, U0s[:,i], tspans[i], p))
  end

  function predict_adjoint(θ)
    global U0s, pargs
    hcat( [Array(concrete_solve(probs[i], Tsit5(), U0s[:,i], θ[6:end], saveat=ts.+(i-1))) for i in 1:pargs["segments"]]... )    # θ[1:5]
  end

  function loss_adjoint(θ)
    pred = predict_adjoint(θ)
    loss = 0.0
    for idx in 1:size(pred,2)
      loss = 10*(odo[idx,1]-pred[1,idx])^2 +
             10*(odo[idx,2]-pred[2,idx])^2 +
             5*(cos(odo[idx,3])-pred[3,idx])^2 +
             5*(sin(odo[idx,3])-pred[4,idx])^2 +
             2*(bodySpeed[idx]-pred[5,idx])^2 +
             (1-pred[3,idx]^2-pred[4,idx]^2)^2
    end
    loss
  end

  θlocal = Float32[U0s[:,1];p]

  # l = loss_adjoint(θlocal)
  # cb(θlocal,l)

  res = DiffEqFlux.sciml_train(loss_adjoint, θlocal, ADAM(α), cb = cb, maxiters=maxiters)
  # res = DiffEqFlux.sciml_train(loss_adjoint, θ, BFGS(initial_stepnorm=0.01), cb = cb)

  return res
end



function trainGeneration(models::Vector{FastChain},
                         u0_all;
                         maxiters::Int=50,
                         MC::Int=10,
                         α=0.05/(10*rand()))

  TASKS = Vector{Any}(undef,MC)
  for i in 1:MC
    TASKS[i] = Threads.@spawn doTraining(models[i], u0_all, maxiters=maxiters, α=α)
  end

  RES = Vector{Any}(undef,MC)
  PP = Vector{Vector{Float32}}(undef, MC)
  CO = zeros(MC)
  for i in 1:MC
    RES[i] = fetch(TASKS[i])
    CO[i] = RES[i].minimum
    PP[i] = deepcopy(RES[i].minimizer)
    @show i, RES[i].minimum
  end


  # TASKS = Vector{Any}(undef,MC)
  # RES = Vector{Any}(undef,MC)
  # GC.gc()
  ord = sortperm(CO)
  return PP[ord], CO[ord]
end



function main(vpdata, ode_data, usrmodel::FastChain; MC::Int=10, maxiters::Int=50)
  global ts, tspan, U0s

  t0 = tspan[1]
  # t0 = vpdata["time"][1] |> Float32

  # x_i, y_i, cosθ_i, sinθ_i, vx_i
  # u0 = Float32[0.; 0.; 1.; 0.; 0.]
  # u0 = zeros(Float32, 4) #Float32(1.1)


  ## Specialized Neural model with time dependent inputs==========================
  models = Vector{FastChain}(undef, MC)
  for thr in 1:MC
    models[thr] = deepcopy(usrmodel)
  end


  PPs, COs = trainGeneration(models, U0s; maxiters=maxiters, MC=MC)

  return PPs, COs
end


# size of pose to pose segment
START=1
STOP=pargs["epochs"]
ode_data = odo[START:STOP,:]' .|> Float32
datasize = size(ode_data,2)
tspan = (Float32(vpdata["time"][1]), Float32(vpdata["time"][datasize]))
ts = range(tspan[1],tspan[2],length=datasize)


# 7 inputs are x, y, cθ, sθ, speed, speed, steer
usrmdl = FastChain(FastDense(7,20,tanh),
                   FastDense(20,20,tanh),
                   FastDense(20,5,tanh))
#


# plot(odo[START:STOP,1], odo[START:STOP,2])
# plot(odo[:,1], odo[:,2])
# plot(bodySpeed[START:STOP])


# test
ptest = doTraining(usrmdl, U0s, maxiters=100, α=0.01)

PPs, COs = main(vpdata, ode_data, usrmdl, MC=pargs["individuals"], maxiters=pargs["maxiters"])


# GC.gc()



##==============================================================================
## Plot Store result


dtim = now()
resultsFolder = ENV["HOME"]*"/Documents/networks/$dtim/"
mkpath(resultsFolder)


fid = open(resultsFolder*"model.txt", "w")
for i in 1:length(usrmdl.layers)
  ll = usrmdl.layers[i]
  println(fid, "layer $i")
  println(fid, "in: $(ll.in)")
  println(fid, "out: $(ll.out)")
  println(fid, "act: $(ll.σ)")
end
close(fid)

PPm = hcat(PPs...)
# COs

writedlm(resultsFolder*"loss.txt", COs)
writedlm(resultsFolder*"params.txt", PPm')

heatmap(PPm)
savefig(resultsFolder*"heatmap.svg")


function dudt_test(u,p,t)
  global usrmdl, tspan, vpdata
  tidx = round(Int, (t-tspan[1])*40+1) # interpolation approach has issues with train
  tidx = 1 < tidx ? tidx : 1
  sp = vpdata["speed"][tidx, 1] + 1e-6*randn()
  st = vpdata["steering"][tidx, 1]
  input = [u; sp; st]
  usrmdl(input, p)
end

u0_test = Float32[ode_data[:,1]; 0.]
prob_test = ODEProblem(dudt_test, u0_test, tspan) #, p

function predict_adjoint_test(θ)
  Array(  concrete_solve(prob_test, Tsit5(), θ[1:5], θ[6:end], saveat=ts)  )
end


for i in 1:length(COs)

XX = predict_adjoint_test(PPs[i])

DRx = [odo[START:STOP,1]';XX[1,:]']'
DRy = [odo[START:STOP,2]';XX[2,:]']'
DRv = [bodySpeed[START:STOP]';XX[5,:]']'

DRct = [cos.(odo[START:STOP,3]');XX[3,:]']'
DRst = [sin.(odo[START:STOP,3]');XX[4,:]']'

fn = plot(DRx, DRy, fmt=:svg)

savefig(resultsFolder*"result_xy_$i.svg")

fn = plot(DRv, fmt=:svg)
savefig(resultsFolder*"result_v_$i.svg")

fn = plot(DRct, fmt=:svg)
savefig(resultsFolder*"result_costh_$i.svg")
fn = plot(DRst, fmt=:svg)
savefig(resultsFolder*"result_sinth_$i.svg")

# plot(XX[1:2,:]')
# plot(XX[1,:], XX[2,:])

end





#
