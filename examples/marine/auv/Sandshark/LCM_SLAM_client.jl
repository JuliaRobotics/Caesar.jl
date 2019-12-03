# AUV msg slam client model

using Distributed
addprocs(4)

using DistributedFactorGraphs
using Caesar, RoME
@everywhere using Caesar, RoME
using LCMCore, BotCoreLCMTypes
using Dates
using DataStructures

# import RoME: nextPose

# Precompile large swath of the solver functions
warmUpSolverJIT()
warmUpSolverJIT() # twice to ensure larger footprint for multiprocess



## middleware handler (callback) functions

function lbl_hdlr(channel::String, msgdata::pose_t, dashboard::Dict, lblDict::Dict)
  msgtime = unix2datetime(msgdata.utime*1e-6)
  dashboard[:lastMsgTime] = msgtime
  lblDict[dashboard[:lastMsgTime]] = (msgdata.pos[1:2] |> deepcopy)
  nothing
end

function mag_hdlr(channel::String, msgdata::pose_t, dashboard::Dict, magDict::Dict)
  msgtime = unix2datetime(msgdata.utime*1e-6)
  dashboard[:lastMsgTime] = msgtime
  ea = TU.convert(Euler, Quaternion(msgdata.orientation[1],msgdata.orientation[2:4]) )
  magDict[dashboard[:lastMsgTime]] = ea.Y
  nothing
end

function range_hdlr(channel::String, msgdata::raw_t, dfg::AbstractDFG, dashboard::Dict)
  msgtime = unix2datetime(msgdata.utime*1e-6)
  dashboard[:lastMsgTime] = msgtime

  # undo pattern used in GenOdo.jl
  data = Array{Float64,2}(undef, 0,0)
  dashboard[:rangeCount] += 1
  if dashboard[:rangeCount] % dashboard[:RANGESTRIDE] == 0
    data = reshape(reinterpret(Float64, msgdata.data), :, 2)
    # Tuple of msgtime, data, ifadded
    push!( dashboard[:rangesBuffer], (msgtime, data, Bool[false;]) )
    dashboard[:rangeCount] = 0
  end

  ## should a range message be added to the factor graph?
  # should be at least one
  if 0 < length(dashboard[:rangesBuffer])
    # check latest pose against all ranges in buffer
    lastPose = getLastPoses(dfg, number=1)[1]
    lastT = timestamp(getVariable(dfg, lastPose))
    @show rangeTimes = map(x->dashboard[:rangesBuffer][x][1], 1:length(dashboard[:rangesBuffer]))
    @show rangeMask = Bool.(abs.(rangeTimes .- lastT) .< Millisecond(100))
    # @show sum(rangeMask), dashboard[:rangeCount]
    # @assert 0 <= sum(rangeMask) <= 1
    # check if already added, add and set flag
    if 0 < sum(rangeMask) # length(dashboard[:rangesBuffer][rangeIdx])
      rangeIdx = collect(1:length(dashboard[:rangesBuffer]))[rangeMask][1]
      @assert typeof(dashboard[:rangesBuffer]) == CircularBuffer{Tuple{DateTime, Array{Float64,2}, Vector{Bool}}}
      if !dashboard[:rangesBuffer][rangeIdx][3][1]
        bera =     Pose2Point2Range(AliasingScalarSampler(dashboard[:rangesBuffer][rangeIdx][2][:,1],dashboard[:rangesBuffer][rangeIdx  ][  2][:,2], SNRfloor=dashboard[:SNRfloor]))
        addFactor!(dfg, [lastPose; :l1], bera, solvable=0, autoinit=false)
        dashboard[:rangesBuffer][rangeIdx][3][1] = true
      end
    end
  end
  nothing
end

function pose_hdlr(channel::String, msgdata::pose_t, dfg::AbstractDFG, dashboard::Dict)
  dashboard[:lastMsgTime] = unix2datetime(msgdata.utime*1e-6)
  # "accumulateDiscreteLocalFrame! on all RTTs" |> println
  for (vsym, rtt) in dashboard[:rttMpp]
    accumulateDiscreteLocalFrame!(rtt, msgdata.pos[1:3], dashboard[:Qc_odo], 1.0)
  end

  # check if a new pose and factor must be added?
  odoT = unix2datetime(msgdata.utime*1e-6)
  if (dashboard[:odoTime] + dashboard[:poseRate]) < odoT
    @info "pose_hdlr, adding new pose at $odoT"
    # update odo time
    dashboard[:odoTime] = odoT

    # get factor to next pose
    nPose = nextPose(dashboard[:lastPose])
    rttLast = dashboard[:rttMpp][dashboard[:lastPose]]
    # add new variable and factor to the graph
    fctsym = duplicateToStandardFactorVariable(Pose2Pose2,
                                               rttLast,
                                               dfg,dashboard[:lastPose],
                                               nPose,
                                               solvable=0,
                                               autoinit=false  )
    #
    # set timestamp to msg times
    setTimestamp!(getVariable(dfg, nPose), odoT)
    # delete rtt unless used by real time prediction
    if dashboard[:lastPose] != dashboard[:rttCurrent][1]
      delete!(dashboard[:rttMpp], dashboard[:lastPose])
    end

    # update last pose to new pose
    dashboard[:lastPose] = nPose

    # create new rtt on new variable
    nRtt = Symbol(string("deadreckon_",string(nPose)[2:end]))
    addVariable!(dfg, nRtt, Pose2, solvable=0)
    drec = MutablePose2Pose2Gaussian(MvNormal(zeros(3), Matrix{Float64}(LinearAlgebra.I, 3,3)))
    addFactor!(dfg, [nPose; nRtt], drec, solvable=0, autoinit=false)
    dashboard[:rttMpp][nPose] = drec
    put!(dashboard[:solvables], [nPose; fctsym])
    dashboard[:poseStride] += 1
    # separate check for LCMLog handling
    dashboard[:doDelay] && 10 <= dashboard[:poseStride] ? (dashboard[:canTakePoses] = 0) : nothing
  end

  nothing
end

function initializeAUV_noprior(dfg::AbstractDFG, dashboard::Dict)
  addVariable!(dfg, :x0, Pose2)

  # Pinger location is [17; 1.8]
  addVariable!(dfg, :l1, Point2, solvable=0)
  beaconprior = PriorPoint2( MvNormal([17; 1.8], Matrix(Diagonal([0.1; 0.1].^2)) ) )
  addFactor!(dfg, [:l1], beaconprior, autoinit=true, solvable=0)

  addVariable!(dfg, :deadreckon_0, Pose2, solvable=0)
  drec = MutablePose2Pose2Gaussian(MvNormal(zeros(3), Matrix{Float64}(LinearAlgebra.I, 3,3)))
  addFactor!(dfg, [:x0; :deadreckon_0], drec, solvable=0, autoinit=false)

  # store current real time tether factor
  dashboard[:rttMpp] = Dict{Symbol,MutablePose2Pose2Gaussian}(:x0 => drec)
  dashboard[:rttCurrent] = (:x0, :deadreckon_0)
  # standard odo process noise levels
  dashboard[:Qc_odo] = Diagonal([0.01;0.01;0.01].^2) |> Matrix

  dashboard[:odoTime] = unix2datetime(0)
  dashboard[:poseRate] = Second(1)
  dashboard[:lastPose] = :x0

  dashboard[:solvables] = Channel{Vector{Symbol}}(100)

  dashboard[:loopSolver] = true

  dashboard[:poseStride] = 0
  dashboard[:canTakePoses] = 1

  dashboard[:rangesBuffer] = CircularBuffer{Tuple{DateTime, Array{Float64,2}, Vector{Bool}}}(10)
  dashboard[:RANGESTRIDE] = 1 # add a range measurement every 3rd pose
  dashboard[:rangeCount] = 0

  dashboard[:SNRfloor] = 0.6

  nothing
end


function manageSolveTree!(dfg::AbstractDFG, dashboard::Dict)

  @info "logpath=$(getLogPath(dfg))"
  getSolverParams(dfg).drawtree = true
  getSolverParams(dfg).qfl = 20
  getSolverParams(dfg).isfixedlag = true
  getSolverParams(dfg).limitfixeddown = true

  # allow async process
  # getSolverParams(dfg).async = true

  # prep with empty tree
  tree = emptyBayesTree()

  # needs to run asynchronously
  @async begin
    while @show length(ls(dfg, :x0, solvable=1)) == 0
      "waiting for prior on x0" |> println
      sleep(1)
    end
    # keep solving
    while dashboard[:loopSolver]
      # add any newly solvables (atomic)
      while !isready(dashboard[:solvables]) && dashboard[:loopSolver]
        @show dashboard[:solvables].data
        sleep(0.5)
      end

      # adjust latest RTT after solve, latest solved
      lastSolved = sortDFG(ls(dfg, r"x\d", solvable=1))[end]
      dashboard[:rttCurrent] = (lastSolved, Symbol("deadreckon_"*string(lastSolved)[2:end]))

      #add any new solvables
      while isready(dashboard[:solvables]) && dashboard[:loopSolver]
        @show tosolv = take!(dashboard[:solvables])
        for sy in tosolv
          # setSolvable!(dfg, sy, 1) # see DFG #221
          # TODO temporary workaround
          getfnc = occursin(r"f", string(sy)) ? getFactor : getVariable
          getfnc(dfg, sy).solvable = 1
        end
      end

      @info "Ensure all new variables initialized"
      ensureAllInitialized!(dfg)

      if 10 <= dashboard[:poseStride]
        # do actual solve
        dashboard[:canTakePoses] = 0
        dashboard[:poseStride] = 0
        tree, smt, hist = solveTree!(dfg, tree)
        # unblock LCMLog reader for next STRIDE segment
        dashboard[:canTakePoses] = 1
        "end of solve cycle" |> println
      else
        sleep(0.2)
      end
    end
  end
end


function main(;lcm=LCM(), logSpeed::Float64=1.0)

  # data containers
  dashboard = Dict{Symbol,Any}()
  lblDict = Dict{DateTime,Vector{Float64}}()
  magDict = Dict{DateTime,Float64}()

  # fg object and initialization
  fg = initfg()
  initializeAUV_noprior(fg, dashboard)

  # prepare the solver in the background
  manageSolveTree!(fg, dashboard)

  # middleware handlers

  # start with LBL and magnetometer
  subscribe(lcm, "AUV_LBL_INTERPOLATED", (c,d)->lbl_hdlr(c,d,dashboard,lblDict), pose_t)
  subscribe(lcm, "AUV_MAGNETOMETER",     (c,d)->mag_hdlr(c,d,dashboard,magDict), pose_t)

  @info "get a starting position and mag orientation"
  while length(lblDict) == 0 || length(magDict) == 0
    handle(lcm)
  end
  @show lblKeys = keys(lblDict) |> collect |> sort
  @show magKeys = keys(magDict) |> collect |> sort
  initPose = [lblDict[lblKeys[1]];magDict[magKeys[1]]]
  dashboard[:odoTime] = lblKeys[1]
  startT = lblKeys[1]

  # add starting prior
  addFactor!(fg, [:x0;], PriorPose2(MvNormal(initPose,Matrix(Diagonal([0.1; 0.1; 0.1].^2)))), autoinit=false)

  @info "Start with the real-time tracking aspect..."
  subscribe(lcm, "AUV_ODOMETRY",         (c,d)->pose_hdlr(c,d,fg,dashboard), pose_t)
  # subscribe(lcm, "AUV_ODOMETRY_GYROBIAS", (c,d)->pose_hdlr(c,d,fg,dashboard), pose_t)
  subscribe(lcm, "AUV_RANGE_CORRL",      (c,d)->range_hdlr(c,d,fg,dashboard), raw_t)
  # subscribe(lcm, "AUV_BEARING_CORRL",callback, raw_t)

  # run handler
  offsetT = now() - startT
  dashboard[:doDelay] = lcm isa LCMLog
  for i in 1:2000
      handle(lcm)
      # slow down in case of using an LCMLog object
      if dashboard[:doDelay]
        while dashboard[:canTakePoses] == 0
          @info "delay for solver to catch up with new poses"
          sleep(0.5)
        end
        dt = (dashboard[:lastMsgTime]-startT)
        nMsgT = startT + typeof(dt)(round(Int, dt.value/logSpeed))
        # @show now() < (nMsgT + offsetT), now(), (nMsgT + offsetT)
        while now() < (nMsgT + offsetT)
          # @info "delay"
          sleep(0.01)
        end
      end
  end
  # close UDP listening on LCM
  close(lcm)

  # close out solver
  dashboard[:loopSolver] = false

  # return last factor graph as built and solved
  return fg, dashboard
end



## Do the actual run
fg, dashboard = main(logSpeed=0.25, lcm=LCMLog(joinpath(ENV["HOME"],"data","sandshark","lcmlog","lcmlog-2019-11-26.01")) )
# fg = main()




timestamp(getVariable(fg, :x0))
setTimestamp!(getVariable(fg, :x0), now())
timestamp(getVariable(fg, :x0))


drawGraph(fg)

allvar = ls(fg, r"x\d") |> sortDFG
hasfc = setdiff(union(map(s->ls(fg, s), ls(fg, :l1))...), [:l1]) |> sortDFG
hasnots = setdiff(allvar, hasfc)


dashboard[:rangesBuffer][1][1]
rT = dashboard[:rangesBuffer][2][1]

pT = timestamp(getVariable(fg, :x1))

abs(rT - pT)


0
## Debugging code below



# drawGraph(fg)
# getSolverParams(fg).dbg = true
# tree, smt, hist = solveTree!(fg, recordcliqs=ls(fg))


#
# using RoMEPlotting
# # drawPoses(fg)
#
# plotPose(fg, :x0)

#
