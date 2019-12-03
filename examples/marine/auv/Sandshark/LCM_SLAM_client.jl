# AUV msg slam client model

using Distributed
addprocs(4)

using DistributedFactorGraphs
using Caesar, RoME
@everywhere using Caesar, RoME
using LCMCore, BotCoreLCMTypes
using Dates
using DataStructures

# Precompile large swath of the solver functions
warmUpSolverJIT()
warmUpSolverJIT() # twice to ensure larger footprint for multiprocess



# bring required utilities and handlers into context
include(joinpath(@__DIR__, "MsgHandlers.jl"))
include(joinpath(@__DIR__, "SandsharkUtils.jl"))


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
