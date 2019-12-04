# all the message handlers for AUV example


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

  # add msg to buffer based on RANGESTRIDE
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
      lastPose = getLastPoses(dfg, number=1)[1]
      lastT = timestamp(getVariable(dfg, lastPose))
    # TODO use findClosestTimestamp instead
    rantim = map(x->(dashboard[:rangesBuffer][x][1],dashboard[:rangesBuffer][x][3][1]), 1:length(dashboard[:rangesBuffer]))
    @show ind, mdt, corrs = findClosestTimestamp([(lastT, lastPose);], rantim)
    # there should be only one correspondence
    @assert corrs <= 1
    # check latest pose against all ranges in buffer
    # @show rangeTimes = map(x->dashboard[:rangesBuffer][x][1], 1:length(dashboard[:rangesBuffer]))
    # @show rangeMask = Bool.(abs.(rangeTimes .- lastT).-dashboard[:rangeClkOffset] .< Millisecond(100))
    # check if already added, add and set flag
    if corrs == 1 && mdt < Millisecond(100)
    # if 0 < sum(rangeMask)
      rangeIdx = ind[2]
      # rangeIdx = collect(1:length(dashboard[:rangesBuffer]))[rangeMask][1]
      # @assert typeof(dashboard[:rangesBuffer]) == CircularBuffer{Tuple{DateTime, Array{Float64,2}, Vector{Bool}}}
      if !dashboard[:rangesBuffer][rangeIdx][3][1]
        bera = Pose2Point2Range(AliasingScalarSampler(dashboard[:rangesBuffer][rangeIdx][2][:,1],dashboard[:rangesBuffer][rangeIdx][2][:,2], SNRfloor=dashboard[:SNRfloor]))
        addFactor!(dfg, [lastPose; :l1], bera, solvable=0, autoinit=false)
        setTimestamp!(getFactor(dfg, lastPose*"l1f1"), msgtime)
        dashboard[:rangesBuffer][rangeIdx][3][1] = true
        #update offset
        @show deltaClk = lastT - dashboard[:rangesBuffer][rangeIdx][1]
        # check that ranges are not lost owing to clock drift
        @assert abs(deltaClk) < Millisecond(100)
        # @show dashboard[:rangeClkOffset] = deltaClk
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
  if (dashboard[:odoTime] + dashboard[:poseRate]) <= odoT
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

#
