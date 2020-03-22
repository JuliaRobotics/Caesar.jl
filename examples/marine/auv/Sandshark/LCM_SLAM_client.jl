# AUV msg slam client model

using Distributed
# addprocs(8)

using Pkg
Pkg.activate(@__DIR__)
pkg"instantiate"
pkg"precompile"

@everywhere begin
    using Pkg
    Pkg.activate(@__DIR__)
end

using Caesar, RoME, DistributedFactorGraphs
@everywhere using Caesar, RoME, DistributedFactorGraphs
using LCMCore, BotCoreLCMTypes
using Dates
using DataStructures
using Logging
using JSON2
using DSP
# using JLD2, FileIO


# mute LCM broadcasting beyond this computer
run(`$(ENV["HOME"])/mutelcm.sh`)


include(joinpath(@__DIR__, "CommonUtils.jl"))

global ODOSCALE = [1.0; 1.0; 1.0]

# bring required utilities and handlers into context
include(joinpath(@__DIR__, "MsgHandlers.jl"))
include(joinpath(@__DIR__, "SandsharkUtils.jl"))



function main(;parsed_args=parse_commandline(),
               lcm=LCM(),
               logSpeed::Float64=parsed_args["speed"],
               iters::Int=parsed_args["iters"],
               dbg::Bool=parsed_args["dbg"],
               savePlotting::Bool=parsed_args["savePlotting"]  )
  #

  # data containers
  dashboard = Dict{Symbol,Any}()
  lblDict = Dict{DateTime,Vector{Float64}}()
  magDict = Dict{DateTime,Float64}()

  # fg object and initialization
  fg = LightDFG{SolverParams}(params=SolverParams())
  # fg = initfg()
  initializeAUV_noprior(fg, dashboard, stride_range=parsed_args["stride_range"], stride_solve=parsed_args["fixedlag"])

  # scale the odometry noise
  dashboard[:odoCov] .*= parsed_args["kappa_odo"]
  dashboard[:savePlotting] = savePlotting

  # store args, dead reckon tether, and lbl values in files.
  Base.mkpath(getLogPath(fg))
  # write the arguments to file
  argsfile = open(joinLogPath(fg,"args.txt"),"w")
  println(argsfile, ARGS)
  println(argsfile, parsed_args)
  close(argsfile)
  argsfile = open(joinLogPath(fg,"args.json"),"w")
  JSON2.write(argsfile, parsed_args)
  close(argsfile)
  DRTLog = open(joinLogPath(fg,"DRT.csv"),"w")
  LBLLog = open(joinLogPath(fg,"LBL.csv"),"w")
  Odolog = open(joinLogPath(fg,"ODO.csv"),"w")
  dirodolog = open(joinLogPath(fg,"DIRODO.csv"),"w")
  rawodolog = open(joinLogPath(fg,"RAWODO.csv"),"w")
  allthtlog = open(joinLogPath(fg,"ALLTHT.csv"),"w")
  posetiminglog = open(joinLogPath(fg,"timing_pose.csv"),"w")
  solvetiminglog = open(joinLogPath(fg,"timing_solve.csv"),"w")

  # prepare the solver in the background
  ST = manageSolveTree!(fg, dashboard, dbg=dbg, timinglog=solvetiminglog, limitfixeddown=parsed_args["limitfixeddown"])

  dshfile = open(joinLogPath(fg,"dashboard_start.json"),"w")
  JSON2.write(dshfile, dashboard)
  close(dshfile)

  # middleware handlers
  # start with LBL and magnetometer
  subscribe(lcm, "AUV_LBL_INTERPOLATED", (c,d)->lbl_hdlr(c,d,fg,dashboard,lblDict,LBLLog), pose_t)
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

  setTimestamp!(fg, :x0, lblKeys[1])
  # setTimestamp!(getVariable(fg, :x0), lblKeys[1])

  # add starting prior
  addFactor!(fg, [:x0;], PriorPose2(MvNormal(initPose,Matrix(Diagonal([0.5; 0.5; 1.0].^2)))), autoinit=false)
  setSolvable!(fg, :x0, 1)
  doautoinit!(fg,:x0)

  @info "Start with the real-time tracking aspect..."
  odoChannel = parsed_args["odoGyroBias"] ? "AUV_ODOMETRY_GYROBIAS" : "AUV_ODOMETRY"
  subscribe(lcm, odoChannel,             (c,d)->pose_hdlr(c,d,fg,dashboard, posetiminglog, DRTLog, Odolog, dirodolog, rawodolog, allthtlog), pose_t)
  # subscribe(lcm, "AUV_ODOMETRY_GYROBIAS", (c,d)->pose_hdlr(c,d,fg,dashboard), pose_t)
  subscribe(lcm, "AUV_RANGE_CORRL",      (c,d)->range_hdlr(c,d,fg,dashboard), raw_t)
  # subscribe(lcm, "AUV_BEARING_CORRL",callback, raw_t)

  # repeat single native odometry
  # subscribe(lcm, "AUV_ODOMETRY",         (c,d)->odo_ensure_hdlr(c,d,fg,dashboard,dirodolog), pose_t)


  recordWTDSH = [true;]
  WTDSH = Vector{Tuple{DateTime, Symbol, Int, HandlerStateMachine, SolverStateMachine, Millisecond, Int}}()
  dashboard[:taskSignals] = @async begin
    # mkpath(getLogPath(fg))
    open(joinLogPath(fg, "dash.csv"), "w") do io
      while recordWTDSH[1]
        line = [now() getLastPoses(fg,filterLabel=r"x\d",number=1)[1] dashboard[:poseStride] dashboard[:solveSettings].canTakePoses dashboard[:solveSettings].solveInProgress dashboard[:realTimeSlack].value length(dashboard[:solveSettings].poseSolveToken.data)]
        # push!(WTDSH, (line...))
        writedlm(io, line, ',')
        sleep(0.005)
      end
    end
  end
  # record trees for later reconstruction into an animation
  dashboard[:taskRecTrees] = @async begin
    Base.mkpath(joinLogPath(fg, "trees"))
    treepath = joinLogPath(fg, "bt.pdf")
    counter = 0
    tcfid = open(joinLogPath(fg, "trees/timestamps.log"),"w")
    while 0 < parsed_args["recordTrees"]
      if ispath(treepath)
        counter += 1
        println(tcfid, "$(dashboard[:lastMsgTime]), $(dashboard[:lastPose]), $counter")
        Base.cp(treepath, joinLogPath(fg, "trees/bt_$(counter).pdf") )
      end
      sleep(1.0/parsed_args["recordTrees"])
    end
    close(tcfid)
  end


  # run handler
  dashboard[:solveSettings].canTakePoses = HSMHandling
  holdTimeRTS = now()
  offsetT = now() - startT
  dashboard[:doDelay] = lcm isa LCMLog
  for i in 1:iters
      handle(lcm)
      # slow down in case of using an LCMLog object
      if dashboard[:doDelay]
        # might be delaying the solver -- ie real time slack grows above zero
        holdTimeRTS = now()
        while 1 < length(dashboard[:solveSettings].poseSolveToken.data)  # !HSMCanContinue(dashboard)
            # while dashboard[:canTakePoses] == HSMBlocking && dashboard[:solveInProgress] == SSMSolving
          flush(DRTLog)
          @info "delay for solver, canTakePoses=$(dashboard[:solveSettings].canTakePoses), tokens=$(dashboard[:solveSettings].poseSolveToken.data), RTS=$(dashboard[:realTimeSlack])"
          # map(x->flush(x.stream), [DRTLog;LBLLog;Odolog;dirodolog;rawodolog;allthtlog;posetiminglog,solvetiminglog])
          sleep(0.2)
        end
        # real time slack update -- this is the compute overrun
        dashboard[:realTimeSlack] += now()-holdTimeRTS
        # playback speed
        dt = (dashboard[:lastMsgTime]-startT)
        nMsgT = startT + typeof(dt)(round(Int, dt.value/logSpeed)) + dashboard[:realTimeSlack]
        while now() < (nMsgT + offsetT)
          flush(DRTLog)
          sleep(0.01)
        end
      end
  end


  # close out solver and other loops
  parsed_args["recordTrees"] = -1.0
  recordWTDSH[1] = false
  dashboard[:solveSettings].loopSolver = false

  # close UDP listening on LCM other file handles
  close(lcm)
  close(DRTLog)
  close(LBLLog)
  close(Odolog)
  close(dirodolog)
  close(rawodolog)
  close(allthtlog)
  close(posetiminglog)
  close(solvetiminglog)

  # return last factor graph as built and solved
  return fg, dashboard, WTDSH, ST
end



# user intentions
parsed_args=parse_commandline()

@show ODOSCALE[1] *= parsed_args["scaleOdoX"]

# ## Uncomment for different defaults in Juno
# parsed_args["iters"] = 15000
# parsed_args["kappa_odo"] = 0.1
# parsed_args["speed"] = 1.0
# parsed_args["recordTrees"] = false


# maybe warmup
# Precompile large swath of the solver functions
if parsed_args["warmupjit"]
  warmUpSolverJIT()
  warmUpSolverJIT() # twice to ensure larger footprint for multiprocess
end


## Do the actual run -- on live UDP traffic
# fg = main()

## Do run on LCM data from file
# lcmlogfile = joinpath(ENV["HOME"],"data","sandshark","lcmlog","lcm-sandshark-med.log")
lcmlogfile = joinpath(ENV["HOME"],"data","sandshark","lcmlog","sandshark-long.lcmlog")
fg, dashboard, wtdsh, ST = main(parsed_args=parsed_args, lcm=LCMLog(lcmlogfile) )

# @save joinLogPath(fg, "dash.jld2") wtdsh
# fid = open(joinLogPath(fg, "dash.json"), "w").
# JSON2.write(fid, wtdsh)
# close(fid)


## moved to CommonUtils.jl
# In case any variables had not been solved yet
ensureAllInitialized!(fg)
saveDFG(fg, joinpath(getLogPath(fg),"fg_final") )


# draw plots in getLogPath(fg)
if parsed_args["genResults"]
  include(joinpath(@__DIR__, "GenerateResults.jl"))
end


## BATCH SOLVE

dontMarginalizeVariablesAll!(fg)
setSolvable!(getVariable(fg, :drt_ref), 0)
tree, smt, hist = solveTree!(fg, maxparallel=1000)

saveDFG(fg, joinpath(getLogPath(fg),"fg_batchsolve") )


if parsed_args["genResults"]
  plb = plotSandsharkFromDFG(fg, drawTriads=false)
  plb |> PDF(joinpath(getLogPath(fg),"traj_batch.pdf"))
end


@show getLogPath(fg)

#
