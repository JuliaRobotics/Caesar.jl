# common utils for applications -- staging area towards RoME itself

using Dates
using DelimitedFiles
using RoME
using DSP
using ArgParse

@enum HandlerStateMachine HSMReady HSMHandling HSMOverlapHandling HSMBlocking
@enum SolverStateMachine SSMReady SSMConsumingSolvables SSMSolving

# Accumulate delta X values using FGOS
# TODO, refactor into RoME
function devAccumulateOdoPose2(DX::Array{Float64,2},
                               X0::Vector{Float64}=zeros(3);
                               P0=1e-3*Matrix(LinearAlgebra.I, 3,3),
                               Qc=1e-6*Matrix(LinearAlgebra.I, 3,3),
                               dt::Float64=1.0,
                               timestamp::DateTime=now() )
  #
  # entries are rows with columns dx,dy,dtheta
  @assert size(DX,2) == 3
  mpp = MutablePose2Pose2Gaussian(MvNormal(X0, P0), timestamp=timestamp )
  nXYT = zeros(size(DX,1), 3)
  for i in 1:size(DX,1)
    RoME.accumulateDiscreteLocalFrame!(mpp,DX[i, :],Qc,dt)
    nXYT[i,:] .= mpp.Zij.μ
  end

  return nXYT
end




function loadResultsDRT(fg)
  drt_data = readdlm(joinLogPath(fg, "DRT.csv"), ',')

  DRTT = DateTime.(drt_data[:,1])
  XX = Float64.(drt_data[:,4])
  YY = Float64.(drt_data[:,5])

  # filter the signals for hard jumps between drt transitions
  responsetype = Lowpass(1.0; fs=50)
  designmethod = FIRWindow(hanning(64))

  # remove those near zero
  mask = 40.0 .< (XX.^2 + YY.^2)

  TTm = DRTT[mask]
  XXm = XX[mask]
  YYm = YY[mask]
  XXf = filt(digitalfilter(responsetype, designmethod), XXm)
  YYf = filt(digitalfilter(responsetype, designmethod), YYm)

  return drt_data, TTm, XXm, YYm, XXf, YYf
end



function parse_commandline()
    s = ArgParseSettings()

    @add_arg_table s begin
        "--plotSeriesBeliefs"
            help = "Glob fg_* archives and draw belief frames as many as is requested, default 0 is for all"
            arg_type = Int
            default = -1
        "--plotSeriesBeliefsNew"
            help = "Glob fg_* archives and draw belief frames as many as is requested, default 0 is for all with background"
            arg_type = Int
            default = -1
        "--reportPoses"
            help = "Generate report on interpose pose factors"
            action = :store_true
        "--reportRanges"
            help = "Generate report on range factors"
            action = :store_true
        "--drawAllRanges"
            help = "Draw ranges as separate images"
            action = :store_true
        "--skip"
            help = "Skip existing images"
            action = :store_true
        "--drawFG"
            help = "Draw factor graph to PDF"
            action = :store_true
        "--reportDir"
            help = "which folder in which to produce results."
            required = false
        "--kappa_odo"
            help = "Scale the odometry covariance"
            arg_type = Float64
            default = 1.0
        "--stride_range"
            help = "Every how many poses to try add a range measurement"
            arg_type = Int
            default = 4
        "--fixedlag"
            help = "FIFO fixed-lag solve length"
            arg_type = Int
            default = 30
        "--magStdDeg"
            help = "Magnetometer standard deviation"
            arg_type = Float64
            default = 5.0
        "--iters", "-i"
            help = "LCM messages to handle"
            arg_type = Int
            default = 10000
        "--speed", "-s"
            help = "Target playback speed for LCMLog"
            arg_type = Float64
            default = 0.2
        "--dbg"
            help = "debug flag"
            action = :store_true
        "--genResults"
            help = "Generate output results"
            action = :store_true
        "--warmupjit"
            help = "Warm up JIT during startup"
            action = :store_true
        "--savePlotting"
            help = "Store factor graph after each pose"
            action = :store_true
        "--odoGyroBias"
            help = "Use gyro biased channel for odometry"
            action = :store_true
        "--limitfixeddown"
            help = "Limit numerical computations for recycled and marginalized cliques during down solve."
            action = :store_true
        "--recordTrees"
            help = "Store copies of the Bayes tree for later animation, sets the rate in sleep(1/rate)"
            arg_type = Float64
            default = -1.0
        "--scaleOdoX"
            help = "Scale the delta X odo parameters."
            arg_type = Float64
            default = 1.0
    end

    return parse_args(s)
end







#
