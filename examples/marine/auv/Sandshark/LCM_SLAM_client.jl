# AUV msg slam client model

using Distributed
addprocs(4)

using DistributedFactorGraphs
using Caesar, RoME
@everywhere using Caesar, RoME
using LCMCore, BotCoreLCMTypes
using Dates
using DataStructures

# mute LCM broadcasting beyond this computer
run(`$(ENV["HOME"])/mutelcm.sh`)


# Precompile large swath of the solver functions
warmUpSolverJIT()
warmUpSolverJIT() # twice to ensure larger footprint for multiprocess


@enum HandlerStateMachine HSMReady HSMHandling HSMOverlapHandling HSMBlocking

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


function main(;lcm=LCM(), logSpeed::Float64=1.0, iters::Int=10000)

  # data containers
  dashboard = Dict{Symbol,Any}()
  lblDict = Dict{DateTime,Vector{Float64}}()
  magDict = Dict{DateTime,Float64}()

  # fg object and initialization
  fg = initfg()
  initializeAUV_noprior(fg, dashboard)

  # prepare the solver in the background
  ST = manageSolveTree!(fg, dashboard, dbg=true)

  # middleware handlers
  # start with LBL and magnetometer
  subscribe(lcm, "AUV_LBL_INTERPOLATED", (c,d)->lbl_hdlr(c,d,dashboard,lblDict), pose_t)
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
  # close UDP listening on LCM
  close(lcm)
  recordWTDSH[1] = false

  # close out solver
  dashboard[:loopSolver] = false

  # return last factor graph as built and solved
  return fg, dashboard, WTDSH, ST
end



## Do the actual run -- on traffic
# fg = main()

## from file
# fg, dashboard = main(logSpeed=0.25, lcm=LCMLog(joinpath(ENV["HOME"],"data","sandshark","lcmlog","lcmlog-2019-11-26.01")) ) # bad ranges
fg, dashboard, wtdsh, ST = main(logSpeed=0.2, iters=25000, lcm=LCMLog(joinpath(ENV["HOME"],"data","sandshark","lcmlog","lcm-sandshark-med.log")) )
# fg, dashboard = main(logSpeed=0.25, lcm=LCMLog(joinpath(ENV["HOME"],"data","sandshark","lcmlog","sandshark-long.lcmlog")) )





##  Draw trajectory & Analyze solve run


plb = plotSandsharkFromDFG(fg)
# pla = drawPosesLandmarksAndOdo(fg, ppbrDict, navkeys, X, Y, lblkeys, lblX, lblY)


plc = plotFrontendTiming(wtdsh)


# Look at range factors separately
reportFactors(fg, Pose2Point2Range)
reportFactors(fg, Pose2Pose2)

## draw fg

drawGraph(fg)


plotPose(fg, :x29)

## Do separate solve

# Check how long not solvable tail is?

map(x->(x,isSolvable(getVariable(fg,x))), getLastPoses(fg, filterLabel=r"x\d", number=15))


# set all to solvable=1

map(x->setSolvable!(fg, x, 1), setdiff(ls(fg), ls(fg, r"drt_\d")))
map(x->setSolvable!(fg, x, 1 ),[lsf(fg, Pose2Pose2);
                                lsf(fg, Pose2Point2Range);
                                lsfPriors(fg)] )
#

ensureAllInitialized!(fg)

# getSolverParams(fg).async = true
tree, smt, hist = solveTree!(fg)





## BATCH SOLVE

dontMarginalizeVariablesAll!(fg)

tree, smt, hist = solveTree!(fg)



### DEV, disengage older variables and factors via solvable=0




tree, smt, hist = solveTree!(fg)



#
