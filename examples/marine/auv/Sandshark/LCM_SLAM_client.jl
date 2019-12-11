# AUV msg slam client model

using Distributed
addprocs(4)

using DistributedFactorGraphs
using Caesar, RoME
@everywhere using Caesar, RoME
using LCMCore, BotCoreLCMTypes
using Dates
using DataStructures
using Logging

using ArgParse

# mute LCM broadcasting beyond this computer
run(`$(ENV["HOME"])/mutelcm.sh`)


# Precompile large swath of the solver functions
warmUpSolverJIT()
warmUpSolverJIT() # twice to ensure larger footprint for multiprocess


@enum HandlerStateMachine HSMReady HSMHandling HSMOverlapHandling HSMBlocking



function parse_commandline()
    s = ArgParseSettings()

    @add_arg_table s begin
        "--kappa_odo"
            help = "Scale the odoemtry covariance"
            arg_type = Float64
            default = 1.0
        "--iters", "-i"
            help = "LCM messages to handle"
            arg_type = Int
            default = 5000
        "--speed", "-s"
            help = "Target playback speed for LCMLog"
            arg_type = Float64
            default = 1.0
        "--dbg"
            help = "debug flag"
            action = :store_true
        # "arg1"
        #     help = "a positional argument"
        #     required = true
    end

    return parse_args(s)
end


# bring required utilities and handlers into context
include(joinpath(@__DIR__, "MsgHandlers.jl"))
include(joinpath(@__DIR__, "SandsharkUtils.jl"))
include(joinpath(@__DIR__, "Plotting.jl"))

# list all cases in which message handling can continue
# return true if MSG handler can continue
function HSMCanContinue(dashboard::Dict)::Bool
  if dashboard[:canTakePoses] == HSMReady && dashboard[:solveInProgress] == SSMSolving ||
     dashboard[:canTakePoses] == HSMHandling && dashboard[:solveInProgress] == SSMSolving ||
     dashboard[:canTakePoses] == HSMHandling && dashboard[:solveInProgress] == SSMReady ||
     dashboard[:canTakePoses] == HSMOverlapHandling && dashboard[:solveInProgress] == SSMSolving ||
     dashboard[:canTakePoses] == HSMReady && dashboard[:solveInProgress] == SSMConsumingSolvables ||
     dashboard[:canTakePoses] == HSMHandling && dashboard[:solveInProgress] == SSMConsumingSolvables ||
     dashboard[:canTakePoses] == HSMOverlapHandling && dashboard[:solveInProgress] == SSMConsumingSolvables
    #
    return true
  end
  return false
end


function main(;parsed_args=parse_commandline(),
               lcm=LCM(),
               logSpeed::Float64=parsed_args["speed"],
               iters::Int=parsed_args["iters"],
               dbg::Bool=parsed_args["dbg"] )
  #

  # data containers
  dashboard = Dict{Symbol,Any}()
  lblDict = Dict{DateTime,Vector{Float64}}()
  magDict = Dict{DateTime,Float64}()

  # fg object and initialization
  fg = initfg()
  initializeAUV_noprior(fg, dashboard)

  # scale the odometry noise
  dashboard[:odoCov] .*= parsed_args["kappa_odo"]

  # prepare the solver in the background
  ST = manageSolveTree!(fg, dashboard, dbg=dbg)

  # middleware handlers
  # start with LBL and magnetometer
  subscribe(lcm, "AUV_LBL_INTERPOLATED", (c,d)->lbl_hdlr(c,d,fg,dashboard,lblDict), pose_t)
  subscribe(lcm, "AUV_MAGNETOMETER",     (c,d)->mag_hdlr(c,d,fg,dashboard,magDict), pose_t)

  @info "get a starting position and mag orientation"
  while length(lblDict) == 0 || length(magDict) == 0
    handle(lcm)
  end
  lblKeys = keys(lblDict) |> collect |> sort
  magKeys = keys(magDict) |> collect |> sort
  initPose = [lblDict[lblKeys[1]];magDict[magKeys[1]]]
  dashboard[:odoTime] = lblKeys[1]
  @show startT = lblKeys[1]

  setTimestamp!(getVariable(fg, :x0), lblKeys[1])

  # add starting prior
  addFactor!(fg, [:x0;], PriorPose2(MvNormal(initPose,Matrix(Diagonal([0.1; 0.1; 0.1].^2)))), autoinit=false)

  @info "Start with the real-time tracking aspect..."
  subscribe(lcm, "AUV_ODOMETRY",         (c,d)->pose_hdlr(c,d,fg,dashboard), pose_t)
  # subscribe(lcm, "AUV_ODOMETRY_GYROBIAS", (c,d)->pose_hdlr(c,d,fg,dashboard), pose_t)
  subscribe(lcm, "AUV_RANGE_CORRL",      (c,d)->range_hdlr(c,d,fg,dashboard), raw_t)
  # subscribe(lcm, "AUV_BEARING_CORRL",callback, raw_t)


  recordWTDSH = [true;]
  WTDSH = Vector{Tuple{DateTime, Symbol, Int, HandlerStateMachine, SolverStateMachine, Millisecond, Int}}()
  @async begin
    while recordWTDSH[1]
      push!(WTDSH, (now(), getLastPoses(fg,filterLabel=r"x\d",number=1)[1],dashboard[:poseStride],dashboard[:canTakePoses],dashboard[:solveInProgress],dashboard[:realTimeSlack],length(dashboard[:poseSolveToken].data) ))
      sleep(0.005)
    end
  end

  # run handler
  dashboard[:canTakePoses] = HSMHandling
  holdTimeRTS = now()
  offsetT = now() - startT
  dashboard[:doDelay] = lcm isa LCMLog
  for i in 1:iters
      handle(lcm)
      # slow down in case of using an LCMLog object
      if dashboard[:doDelay]
        # might be delaying the solver -- ie real time slack grows above zero
        holdTimeRTS = now()
        while 1 < length(dashboard[:poseSolveToken].data)  # !HSMCanContinue(dashboard)
            # while dashboard[:canTakePoses] == HSMBlocking && dashboard[:solveInProgress] == SSMSolving
          @info "delay for solver, canTakePoses=$(dashboard[:canTakePoses]), solveInPrg.=$(dashboard[:solveInProgress]), RTS=$(dashboard[:realTimeSlack])"
          sleep(0.5)
        end
        # real time slack update -- this is the compute overrun
        dashboard[:realTimeSlack] += now()-holdTimeRTS
        # playback speed
        dt = (dashboard[:lastMsgTime]-startT)
        nMsgT = startT + typeof(dt)(round(Int, dt.value/logSpeed)) + dashboard[:realTimeSlack]
        while now() < (nMsgT + offsetT)
          sleep(0.01)
        end
      end
  end

  # close UDP listening on LCM other file handles
  close(lcm)

  # close out solver and other loops
  dashboard[:loopSolver] = false
  recordWTDSH[1] = false

  # return last factor graph as built and solved
  return fg, dashboard, WTDSH, ST
end



## Do the actual run -- on traffic
# fg = main()

# logSpeed=0.2, iters=50000,

## from file
# fg, dashboard = main(logSpeed=0.25, lcm=LCMLog(joinpath(ENV["HOME"],"data","sandshark","lcmlog","lcmlog-2019-11-26.01")) ) # bad ranges
fg, dashboard, wtdsh, ST = main(lcm=LCMLog(joinpath(ENV["HOME"],"data","sandshark","lcmlog","lcm-sandshark-med.log")) )
# fg, dashboard = main(logSpeed=0.25, lcm=LCMLog(joinpath(ENV["HOME"],"data","sandshark","lcmlog","sandshark-long.lcmlog")) )



## draw fg
drawGraph(fg, filepath=joinpath(getLogPath(fg),"fg.pdf"), show=false)


##  Draw trajectory & Analyze solve run


plb = plotSandsharkFromDFG(fg)
plb |> PDF(joinpath(getLogPath(fg),"traj.pdf"))
# pla = drawPosesLandmarksAndOdo(fg, ppbrDict, navkeys, X, Y, lblkeys, lblX, lblY)


plc = plotFrontendTiming(wtdsh)
plc |> PDF(joinpath(getLogPath(fg),"timing.pdf"),75cm,25cm)
# accumulated real time slack
wtt = (x->x[6]).(wtdsh)
wtt[end]




# set all to solvable=1

map(x->setSolvable!(fg, x, 1), setdiff(ls(fg), ls(fg, r"drt_\d")))
map(x->setSolvable!(fg, x, 1 ),[lsf(fg, Pose2Pose2);
                                lsf(fg, Pose2Point2Range);
                                lsfPriors(fg)] )
#
ensureAllInitialized!(fg)

# Check how long not solvable tail is?
# map(x->(x,isSolvable(getVariable(fg,x))), getLastPoses(fg, filterLabel=r"x\d", number=15))


## add reference layer
posesyms = ls(fg, r"x\d") |> sortDFG
XX = (x->(getVal(solverData(getVariable(fg, x), :lbl))[1])).(posesyms)
YY = (x->(getVal(solverData(getVariable(fg, x), :lbl))[2])).(posesyms)
pl = Gadfly.layer(x=XX, y=YY, Geom.path, Theme(default_color=colorant"magenta"))
union!(plb.layers, pl)
plb |> PDF(joinpath(getLogPath(fg),"traj_ref.pdf"))




saveDFG(fg, joinpath(getLogPath(fg),"fg_final") )




## Look at factors separately
reportFactors(fg, Pose2Point2Range, show=false)
# reportFactors(fg, Pose2Pose2)


## BATCH SOLVE

# dontMarginalizeVariablesAll!(fg)

# tree, smt, hist = solveTree!(fg)




## Plot reference poses



#
