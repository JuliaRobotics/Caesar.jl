# AUV msg slam client model

using Distributed
addprocs(4)

using Caesar, RoME
@everywhere using Caesar, RoME
using LCMCore, BotCoreLCMTypes
using Dates


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

function pose_hdlr(channel::String, msgdata::pose_t, fg::AbstractDFG, dashboard::Dict)

  "accumulateDiscreteLocalFrame!" |> println
  accumulateDiscreteLocalFrame!(dashboard[:rttMpp], msgdata.pos[1:3], dashboard[:Qc_odo], 1.0)

  nothing
end

function initializeAUV_noprior(dfg::AbstractDFG, dashboard::Dict)
  addVariable!(dfg, :x0, Pose2)

  # Pinger location is [17; 1.8]
  addVariable!(dfg, :l1, Point2)
  beaconprior = PriorPoint2( MvNormal([17; 1.8], Matrix(Diagonal([0.1; 0.1].^2)) ) )
  addFactor!(dfg, [:l1], beaconprior, autoinit=true)

  addVariable!(dfg, :deadreckon_x0, Pose2, solvable=0)
  drec = MutablePose2Pose2Gaussian(MvNormal(zeros(3), Matrix{Float64}(LinearAlgebra.I, 3,3)))
  addFactor!(dfg, [:x0; :deadreckon_x0], drec, solvable=0, autoinit=false)

  # store current real time tether factor
  dashboard[:rttMpp] = drec
  dashboard[:rttVarSym] = :deadreckon_x0
  # standard odo process noise levels
  dashboard[:Qc_odo] = Diagonal([0.01;0.01;0.01].^2) |> Matrix

  nothing
end

function main()

  # data containers
  dashboard = Dict{Symbol,Any}()
  lblDict = Dict{DateTime,Vector{Float64}}()
  magDict = Dict{DateTime,Float64}()

  # fg object and initialization
  fg = initfg()
  initializeAUV_noprior(fg, dashboard)

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

  # add starting prior
  addFactor!(fg, [:x0;], PriorPose2(MvNormal(initPose,Matrix(Diagonal([0.1; 0.1; 0.1].^2)))), autoinit=false)

  @info "Start with the real-time tracking aspect..."
  subscribe(lcm, "AUV_ODOMETRY",         (c,d)->pose_hdlr(c,d,fg,dashboard), pose_t)
  # subscribe(lcm, "AUV_ODOMETRY_GYROBIAS",callback, pose_t)
  # subscribe(lcm, "AUV_RANGE_CORRL",bytes)
  # subscribe(lcm, "AUV_BEARING_CORRL",bytes)

  getSolverParams(fg).async = true
  tree = wipeBuildNewTree!(fg)

  # tree, smt, hist = solveTree!(fg, tree)

  for i in 1:100
      handle(lcm)
  end
  close(lcm)

  return fg
end





fg = main()




#
