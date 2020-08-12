# https://diffeqflux.sciml.ai/dev/examples/LV-Univ/

# using Revise

using DiffEqFlux, Flux, Optim, OrdinaryDiffEq, Plots
using DataInterpolations
using RoME

import DistributedFactorGraphs: getVariableLabelNumber, findFactorsBetweenNaive

using JLD2
using Dates
using Printf

# import IncrementalInference: accumulateFactorChain

# For DistributedFactorGraphs containing FluxModelsPose2Pose2.
# extract pose positions and times as well as joyveldata from flux factors.
function extractTimePoseJoyData(fg::AbstractDFG)
  poses = ls(fg, r"x\d") |> sortDFG
  fcts = lsf(fg, FluxModelsPose2Pose2) |>sortDFG

  JOYDATA = zeros(length(fcts),25,4)
  POSDATA = zeros(length(poses),3,100)
  TIMES = zeros(length(poses))

  T0 = getTimestamp(getVariable(fg, poses[1]))
  POSDATA[1,:,:] .= getVal(fg, poses[1])
  for i in 1:length(poses)-1
    POSDATA[i+1,:,:] .= getVal(fg, poses[i+1])
    JOYDATA[i,:,:] = getFactorType(fg, fcts[i]).joyVelData
    TIMES[i+1] = (getTimestamp(getVariable(fg, poses[i+1])) - T0).value * 1e-3
  end

  return TIMES, POSDATA, JOYDATA
end



##

fg = initfg()
loadDFG("/tmp/caesar/2020-05-11T02:25:53.702/fg_204_resolve.tar.gz", Main, fg)


# TIMES, POSES, JOYDATA = extractTimePoseJoyData(fg)
#
# # make sure its Float32
# TIMES, POSES, JOYDATA = Float32.(TIMES), Float32.(POSES), Float32.(JOYDATA)

# get all velocities



# structure all data according to poses

## get timestamps

MAXADI = 10
vsyms = ls(fg, r"x\d") |> sortDFG

# store all timestamps in a dictionary
timestamps = Dict{Symbol, DateTime}()
vsyms .|> x->(timestamps[x]=getTimestamp(getVariable(fg, x)));

timestamps


## Get chords info

# chords = RoME.assembleChordsDict(fg, MAXADI=10)
JLD2.@load "/tmp/caesar/2020-05-11T02:25:53.702/chords_10.jld2" chords


## Make sure velocities are set

whichVelIdx = ones(Int, 100)
measSeq = collect(1:100)

setNaiveFracAll!(fg, 1.0)
meas = freshSamples(fg, :x0x1f1, 100)
setNaiveFracAll!(fg, 0.0)
for fs in fsyms, idx in 1:100
  fill!(whichVelIdx,1)
  whichVelIdx[idx] = 2
  dummy = approxConv(fg, fs, :x1, (meas[1],whichVelIdx,measSeq) )
  vecjoy = getFactorType(fg, fs).joyVelData
end

## Store joystick values in composite

# datastructure to be populated
joysticks = Dict{Symbol,Dict{Symbol,Vector{Matrix{Float32}}}}()

whichVelIdx = ones(Int, 1)
measSeq = ones(Int, 100)
lastPoseNum = getVariableLabelNumber(vsyms[end])
for from in vsyms[1:end-1]
  SRT = getVariableLabelNumber(from)
  joysticks[from] = Dict{Symbol, Vector{Matrix{Float32}}}()
  maxadi = lastPoseNum - getVariableLabelNumber(from)
  maxadi = MAXADI < maxadi ? MAXADI : maxadi
  for adi in 1:maxadi
      to = Symbol("x",getVariableLabelNumber(from)+adi)
      # init the to field
      if !haskey(joysticks[from], to)
        joysticks[from][to]=Vector{Matrix{Float32}}(undef, 100)
        for i in 1:100  joysticks[from][to][i]=zeros(Float32,0,4); end
      end
      fsyms = findFactorsBetweenNaive(fg, from, to)
      # vecjoy = getFactorType.(fg, fsyms) .|> x->x.joyVelData
      # joysticks[from][to] = length(vecjoy)== 1 ? vecjoy[1] : vcat(vecjoy...)
      for fs in fsyms
        intermTo = getVariableOrder(fg, fs)[end]
        setNaiveFracAll!(fg, 1.0)
        meas = freshSamples(fg, fs, 1)
        setNaiveFracAll!(fg, 0.0)
        for idx in 1:100
          # fill!(whichVelIdx,1) # only if doing all 100
          whichVelIdx[1] = 2
          fill!(measSeq, idx)
          dummy = approxConv(fg, fs, intermTo, (meas[1],whichVelIdx,measSeq) )
          vecjoy = getFactorType(fg, fs).joyVelData
          joysticks[from][to][idx] = vcat(joysticks[from][to][idx], vecjoy)
        end
        @show "done $fs"
      end
  end
end

joysticks

# JLD2.@save "/tmp/caesar/2020-05-11T02:25:53.702/joysticks_10_alpha.jld2" joysticks

# joysticks_ = deepcopy(joysticks)

##

chords[:x0][:x1]
joysticks[:x0][:x1]

chords[:x2][:x9]
joysticks[:x2][:x9]


datasets = [ chords[:x0][:x1]]


##

# take in [X;u] = [x;y;ct;st;dx;dy;thr;str]
# output X
model_g = FastChain(FastDense( 8, 16, tanh),
                    FastDense(16, 16, tanh),
                    FastDense(16, 6) )
#

# The model weights are destructured into a vector of parameters
p_model = initial_params(model_g)
n_weights = length(p_model)

# # Parameters of the second equation (linear dynamics)
# p_system = Float32[0.0; 0.0, vx0, vy0]

p_all = p_model # [p_model; p_system]

# parameters to be optimized
θ = Float32.(p_all)


## quasi-static parameters

# START = 3
#
# vx0, vy0 = JOYDATA[START,1,3], JOYDATA[START,1,4] # 0, 0
# x0 = [0.0;0;1.0;0.0;vx0;vy0]

# dx/dt = Fx + g(x,u)
function dxdt_odo!( dx, x, p, t, model_g, thr_itp, str_itp )
  # Destructure the parameters
  @assert length(p) == n_weights "dxdt_odo! model weights $(length(p)) do not match $n_weights"

  # Getting the throttle and steering inputs (from interpolators)
  thr = thr_itp(t)
  str = str_itp(t)

  # Update in place
  # Velocity model dynamics and TBD odometry model
  # The neural network outputs odometry from joystick values
  dx[1:6] .= model_g([x;thr;str], p)
  dx[1:2] .+= x[5:6] # try remove this structure later
  dx
end
# model_weights = p[1:n_weights]
# state = p[(end-3):end]


## loss function elements

# elems = [[:x2];
#          [:x2;:x3;];
#          [:x2;:x3;:x4];]
SRT, STP = 2, 3
elems = [[Symbol("x",i) for i in SRT:j+1] for j in SRT:STP-1]


##

idx = 1
vx0, vy0 = 0.0, 0.0
x0 = [0.0;0;1.0;0.0;vx0;vy0] .|> Float32

STEP = 1
from = elems[STEP][1]
to = elems[STEP][end]
next = Symbol("x",getVariableLabelNumber(to)+1)
# haskey(joysticks[from], next) || break
DT = (timestamps[to]-timestamps[from]).value/1000 |> Float32
tspan = (0.0f0, DT)
tsteps = range(tspan[1],tspan[2],length=25*(length(elems[STEP])-1))

# use interpolators
throttle_itp = LinearInterpolation( joysticks[from][to][:,1], tsteps )
steering_itp = LinearInterpolation( joysticks[from][to][:,2], tsteps )

prob_odo = ODEProblem((dx,x,p,t)->dxdt_odo!(dx,x,p,t,model_g,throttle_itp,steering_itp), x0, tspan, θ)
sol_odo = solve(prob_odo, Tsit5(), p=θ, saveat=tsteps, abstol=1e-6,reltol=1e-6 )

predArr = Array(sol_odo)
predicted = predArr[:,end]

val = chords[from][to][2][:,idx]
target = [val[1];val[2];cos(val[3]);sin(val[3]);]

joysticks[from][:x8]


function predict_univ(θ, x0_)
  # solving for both the ODE solution and network parameters
  return Array(solve(prob_odo, Tsit5(), p=θ,
                              saveat=tsteps))
end

function loss_univ(θ)

  # add for loop here for multiple segments
  pred = predict_univ(θ, x0)
  pred_x = pred[1,25:25:end]
  pred_y = pred[2,25:25:end]
  pred_ct = pred[3,25:25:end]
  pred_st = pred[4,25:25:end]
  pred_dxy = pred[5:6,25:25:end]
  # pred_dy = pred[6,25:25:end]

  # wait this does not derotate into relative observed values ()
  meas_x = POSES[START:(START+STEPS),1,1] |> reverse |> diff |> reverse
  meas_y = POSES[START:(START+STEPS),1,2] |> reverse |> diff |> reverse
  # FIXME, wont work beyond pi radians (must also preserve AD)
  meas_t = POSES[START:(START+STEPS),1,3] |> reverse |> diff |> reverse
  meas_ct = cos.(meas_t)
  meas_st = sin.(meas_t)

  # @show meas_x, pred_x
  ret = 0.0

  ret += sum(abs2, meas_x - pred_x)
  ret += sum(abs2, meas_y - pred_y)
  ret += sum(abs2, meas_ct - pred_ct)
  ret += sum(abs2, meas_st - pred_st)
  for i in 1:STEPS
    ret += 0.001*sum(abs2, pred[5,25*i] .- pred[5,((i-1)*25+1):(i*25-1)])
    ret += 0.001*sum(abs2, pred[6,25*i] .- pred[6,((i-1)*25+1):(i*25-1)])
  end

  return ret
end
l = predict_univ(θ)
l = loss_univ(θ)

θ


##


list_plots = []
iter = 0
callback = function (θ, l)
  global list_plots, iter

  if iter == 0
    list_plots = []
  end
  iter += 1

  @printf(stdout, "loss=%1.4E\n", l)

  pred = predict_univ(θ)

  plt = Plots.plot([pred[1];pred[1]+0.1], [pred[2];pred[2]], ylim=(-3, 3), xlim=(-1,5))
  # plt = Plots.plot(predict_univ(θ), ylim = (0, 6))
  push!(list_plots, plt)
  display(plt)
  return false
end


result_univ = DiffEqFlux.sciml_train(loss_univ, θ,
                                     ADAM(0.01),
                                     cb = callback,
                                     maxiters=500)
                                     # BFGS(initial_stepnorm = 0.01),
#

## Lest confirm all is good

result_univ.minimizer
result_univ.minimizer

 # - x0 |> norm


loss_univ(result_univ.minimizer)





####  Dev on interpose values

using RoMEPlotting
Gadfly.set_default_plot_size(35cm,20cm)



dfg = initfg()
loadDFG("/tmp/caesar/2020-05-11T02:25:53.702/fg_204_resolve.tar.gz", Main, dfg)

setNaiveFracAll!(dfg,1.0)

meas, pred = solveFactorMeasurements(dfg, :x0x1f1)

# reportFactors(dfg, FluxModelsPose2Pose2, [:x0x1f1;])

#



# function solveFactorMeasurementsChain(dfg::AbstractDFG, fsyms::Vector{Symbol})
#
#   # FCTS = getFactorType.(dfg, fsyms)
#   FCTS = solveFactorMeasurements.(dfg, fsyms)
#
# end





chords = RoME.assembleChordsDict(dfg, MAXADI=10)


plotFactorValues(chords[:x0][:x1]...)
plotFactorValues(chords[:x1][:x2]...)
plotFactorValues(chords[:x0][:x2]...)



JLD2.@save "/tmp/caesar/2020-05-11T02:25:53.702/chords_10.jld2" chords


using JSON2


str = JSON2.write(chords)
io = open("/tmp/chords10.json", "w")
println(io, str)
close(io)

str = JSON2.write(joysticks)
io = open("/tmp/joysticks10.json", "w")
println(io, str)
close(io)


io = open("/tmp/chords.json","r")
data = read(io)
close(io)

rstr = String(take!(IOBuffer(data)))


ndat = JSON2.read(rstr)


reshape(ndat[:x0][:x1][1], 3,100)







#
