# AUV msg slam client model

using Distributed
addprocs(4)

using DistributedFactorGraphs
using Caesar, RoME
@everywhere using Caesar, RoME
using LCMCore, BotCoreLCMTypes
using Dates

import RoME: nextPose


## middleware handler (callback) functions

function lbl_hdlr(channel::String, msgdata::pose_t, lblDict::Dict)
  lblDict[unix2datetime(msgdata.utime*1e-6)] = msgdata.pos[1:2] |> deepcopy
  nothing
end

function mag_hdlr(channel::String, msgdata::pose_t, magDict::Dict)
  ea = TU.convert(Euler, Quaternion(msgdata.orientation[1],msgdata.orientation[2:4]) )
  magDict[unix2datetime(msgdata.utime*1e-6)] = ea.Y
  nothing
end


function pose_hdlr(channel::String, msgdata::pose_t, dfg::AbstractDFG, dashboard::Dict)
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
    while true
      # add any newly solvables (atomic)
      while !isready(dashboard[:solvables])
        @show dashboard[:solvables].data
        sleep(0.5)
      end

      # adjust latest RTT after solve, latest solved
      lastSolved = sortDFG(ls(dfg, r"x\d", solvable=1))[end]
      dashboard[:rttCurrent] = (lastSolved, Symbol("deadreckon_"*string(lastSolved)[2:end]))

      #add any new solvables
      while isready(dashboard[:solvables])
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

      # do actual solve
      tree, smt, hist = solveTree!(dfg, tree)
      "end of solve cycle" |> println
    end
  end
end


function main()

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
  lcm = LCM()

  # start with LBL and magnetometer
  subscribe(lcm, "AUV_LBL_INTERPOLATED", (c,d)->lbl_hdlr(c,d,lblDict), pose_t)
  subscribe(lcm, "AUV_MAGNETOMETER",     (c,d)->mag_hdlr(c,d,magDict), pose_t)

  @info "get a starting position and mag orientation"
  while length(lblDict) == 0 || length(magDict) == 0
    handle(lcm)
  end
  @show lblKeys = keys(lblDict) |> collect |> sort
  @show magKeys = keys(magDict) |> collect |> sort
  initPose = [lblDict[lblKeys[1]];magDict[magKeys[1]]]
  dashboard[:odoTime] = lblKeys[1]

  # add starting prior
  addFactor!(fg, [:x0;], PriorPose2(MvNormal(initPose,Matrix(Diagonal([0.1; 0.1; 0.1].^2)))), autoinit=false)

  @info "Start with the real-time tracking aspect..."
  subscribe(lcm, "AUV_ODOMETRY",         (c,d)->pose_hdlr(c,d,fg,dashboard), pose_t)
  # subscribe(lcm, "AUV_ODOMETRY_GYROBIAS", (c,d)->pose_hdlr(c,d,fg,dashboard), pose_t)
  # subscribe(lcm, "AUV_RANGE_CORRL",  callback, raw_t)
  # subscribe(lcm, "AUV_BEARING_CORRL",callback, raw_t)

  # run handler
  for i in 1:10000
      handle(lcm)
  end
  close(lcm)

  return fg
end



## Do the actual run
fg = main()


drawGraph(fg)



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
