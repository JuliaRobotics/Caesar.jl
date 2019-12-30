# AUV msg slam client model

using Distributed
addprocs(8)

using DistributedFactorGraphs
using Caesar, RoME
@everywhere using Caesar, RoME
using LCMCore, BotCoreLCMTypes
using Dates
using DataStructures
using Logging
using JSON2
using DSP

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
            help = "Scale the odometry covariance"
            arg_type = Float64
            default = 1.0
        "--stride_range"
            help = "Every how many poses to try add a range measurement"
            arg_type = Int
            default = 4
        "--magStdDeg"
            help = "Magnetometer standard deviation"
            arg_type = Float64
            default = 5.0
        "--iters", "-i"
            help = "LCM messages to handle"
            arg_type = Int
            default = 15000
        "--speed", "-s"
            help = "Target playback speed for LCMLog"
            arg_type = Float64
            default = 0.2
        "--dbg"
            help = "debug flag"
            action = :store_true
        "--recordTrees"
            help = "Store copies of the Bayes tree for later animation, sets the rate in sleep(1/rate)"
            arg_type = Float64
            default = -1.0
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
# function HSMCanContinue(dashboard::Dict)::Bool
#   if dashboard[:canTakePoses] == HSMReady && dashboard[:solveInProgress] == SSMSolving ||
#      dashboard[:canTakePoses] == HSMHandling && dashboard[:solveInProgress] == SSMSolving ||
#      dashboard[:canTakePoses] == HSMHandling && dashboard[:solveInProgress] == SSMReady ||
#      dashboard[:canTakePoses] == HSMOverlapHandling && dashboard[:solveInProgress] == SSMSolving ||
#      dashboard[:canTakePoses] == HSMReady && dashboard[:solveInProgress] == SSMConsumingSolvables ||
#      dashboard[:canTakePoses] == HSMHandling && dashboard[:solveInProgress] == SSMConsumingSolvables ||
#      dashboard[:canTakePoses] == HSMOverlapHandling && dashboard[:solveInProgress] == SSMConsumingSolvables
#     #
#     return true
#   end
#   return false
# end


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
  initializeAUV_noprior(fg, dashboard, stride_range=parsed_args["stride_range"])

  # scale the odometry noise
  dashboard[:odoCov] .*= parsed_args["kappa_odo"]

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

  # prepare the solver in the background
  ST = manageSolveTree!(fg, dashboard, dbg=dbg)

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

  setTimestamp!(getVariable(fg, :x0), lblKeys[1])

  # add starting prior
  addFactor!(fg, [:x0;], PriorPose2(MvNormal(initPose,Matrix(Diagonal([0.5; 0.5; 1.0].^2)))), autoinit=false)

  @info "Start with the real-time tracking aspect..."
  subscribe(lcm, "AUV_ODOMETRY",         (c,d)->pose_hdlr(c,d,fg,dashboard, DRTLog, Odolog, dirodolog, rawodolog), pose_t)
  # subscribe(lcm, "AUV_ODOMETRY_GYROBIAS", (c,d)->pose_hdlr(c,d,fg,dashboard), pose_t)
  subscribe(lcm, "AUV_RANGE_CORRL",      (c,d)->range_hdlr(c,d,fg,dashboard), raw_t)
  # subscribe(lcm, "AUV_BEARING_CORRL",callback, raw_t)

  # repeat single native odometry
  # subscribe(lcm, "AUV_ODOMETRY",         (c,d)->odo_ensure_hdlr(c,d,fg,dashboard,dirodolog), pose_t)


  recordWTDSH = [true;]
  WTDSH = Vector{Tuple{DateTime, Symbol, Int, HandlerStateMachine, SolverStateMachine, Millisecond, Int}}()
  dashboard[:taskSignals] = @async begin
    while recordWTDSH[1]
      push!(WTDSH, (now(), getLastPoses(fg,filterLabel=r"x\d",number=1)[1],dashboard[:poseStride],dashboard[:canTakePoses],dashboard[:solveInProgress],dashboard[:realTimeSlack],length(dashboard[:poseSolveToken].data) ))
      sleep(0.005)
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
          flush(DRTLog)
          @info "delay for solver, canTakePoses=$(dashboard[:canTakePoses]), solveInPrg.=$(dashboard[:solveInProgress]), RTS=$(dashboard[:realTimeSlack])"
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
  dashboard[:loopSolver] = false

  # close UDP listening on LCM other file handles
  close(lcm)
  close(DRTLog)
  close(LBLLog)
  close(Odolog)
  close(dirodolog)
  close(rawodolog)

  # return last factor graph as built and solved
  return fg, dashboard, WTDSH, ST
end



# user intentions
parsed_args=parse_commandline()

# ## Uncomment for different defaults in Juno
parsed_args["iters"] = 30000
parsed_args["speed"] = 1.0
# parsed_args["recordTrees"] = false


## Do the actual run -- on live UDP traffic
# fg = main()

## Do run on LCM data from file
lcmlogfile = joinpath(ENV["HOME"],"data","sandshark","lcmlog","lcm-sandshark-med.log")
# lcmlogfile = joinpath(ENV["HOME"],"data","sandshark","lcmlog","sandshark-long.lcmlog")
fg, dashboard, wtdsh, ST = main(parsed_args=parsed_args, lcm=LCMLog(lcmlogfile) )




## draw fg
drawGraph(fg, filepath=joinpath(getLogPath(fg),"fg.pdf"), show=false)
# drawGraph(fg)


##  Draw trajectory & Analyze solve run


plb = plotSandsharkFromDFG(fg, drawTriads=false)
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

# after all initialized
plb = plotSandsharkFromDFG(fg, drawTriads=false, lbls=false)


## add reference layer
posesyms = ls(fg, r"x\d") |> sortDFG
XX = (x->(getVal(solverData(getVariable(fg, x), :lbl))[1])).(posesyms)
YY = (x->(getVal(solverData(getVariable(fg, x), :lbl))[2])).(posesyms)
pl = Gadfly.layer(x=XX, y=YY, Geom.path, Theme(default_color=colorant"magenta"))
union!(plb.layers, pl)
plb |> PDF(joinLogPath(fg,"traj_ref.pdf"))




saveDFG(fg, joinpath(getLogPath(fg),"fg_final") )



## plot the DEAD RECKON THREAD



drt_data = readdlm(joinLogPath(fg, "DRT.csv"), ',')

XX = Float64.(drt_data[:,3])
YY = Float64.(drt_data[:,4])

# remove those near zero
mask = 40.0 .< (XX.^2 + YY.^2)

# filter the signals for hard jumps between drt transitions
responsetype = Lowpass(20.0; fs=50)
designmethod = FIRWindow(hanning(32))

XXf = XX[mask] #filt(digitalfilter(responsetype, designmethod), XX[mask])
YYf = YY[mask] #filt(digitalfilter(responsetype, designmethod), YY[mask])

pl = Gadfly.layer(x=XXf, y=YYf, Geom.path, Theme(default_color=colorant"green"))
union!(plb.layers, pl)
plb |> PDF(joinLogPath(fg,"traj_ref_drt.pdf"))






odo_data = readdlm(joinLogPath(fg, "ODO.csv"), ',')

XX = Float64.(odo_data[:,3])
YY = Float64.(odo_data[:,4])

# remove those near zero
mask = 40.0 .< (XX.^2 + YY.^2)

XXf = XX #[mask]
YYf = YY #[mask]

pl = Gadfly.layer(x=XXf, y=YYf, Geom.path, Theme(default_color=colorant"black"))
union!(plb.layers, pl)
plb |> PDF(joinLogPath(fg,"traj_ref_drt_odo.pdf"))








odo_data = readdlm(joinLogPath(fg, "RAWODO.csv"), ',')

DX = Float64.(odo_data[:,3:5])

nXYT = devAccumulateOdoPose2(DX, getFactorType(getFactor(fg, :x0f1)).Z.μ)

# remove those near zero for better visualization
mask = 40.0 .< (nXYT[:,1].^2 + nXYT[:,2].^2)

XXf = nXYT[:,1][mask]
YYf = nXYT[:,2][mask]

# pl = Gadfly.plot(x=XXf, y=YYf, Geom.path, Theme(default_color=colorant"black"))
pl = plotTrajectoryArrayPose2(nXYT)
pl |> PDF(joinLogPath(fg,"dirodo.pdf"))

pl = Gadfly.layer(x=XXf, y=YYf, Geom.path, Theme(default_color=colorant"black"))
union!(plb.layers, pl)
plb |> PDF(joinLogPath(fg,"traj_ref_drt_dirodo.pdf"))






## Look at factors separately
reportFactors(fg, Pose2Point2Range, show=false)
# reportFactors(fg, Pose2Pose2, show=false)


## BATCH SOLVE

# dontMarginalizeVariablesAll!(fg)

# tree, smt, hist = solveTree!(fg)




## Plot reference poses



#
# fg, dashboard = main(logSpeed=0.25, lcm=LCMLog(joinpath(ENV["HOME"],"data","sandshark","lcmlog","lcmlog-2019-11-26.01")) ) # bad ranges

# find . -type f -name '*.pdf' -print0 |
#   while IFS= read -r -d '' file
#     do convert -verbose -density 500 -quality 100 "${file}" "png/${file%.*}.png"
#   done

# ls nobytes/ | sed 's/bt_//g' | sed 's/.pdf//g' | xargs -L1
