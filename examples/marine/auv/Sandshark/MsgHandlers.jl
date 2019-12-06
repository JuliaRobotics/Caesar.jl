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
  # at some point there was a bug where range data was not in lcmlog as expected -- it was not understood, hence the assert
  @assert msgdata.length >= 130000

  # add msg to buffer based on RANGESTRIDE
  data = Array{Float64,2}(undef, 0,0)
  dashboard[:rangeCount] += 1
  if dashboard[:rangeCount] % dashboard[:RANGESTRIDE] == 0
    dashboard[:lastRangeMsg] = deepcopy(msgdata)
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
    # check latest pose against all ranges in buffer
    rantim = map(x->(dashboard[:rangesBuffer][x][1],dashboard[:rangesBuffer][x][3][1]), 1:length(dashboard[:rangesBuffer]))
    @show ind, mdt, corrs = findClosestTimestamp([(lastT, lastPose);], rantim)
    # there should be only one correspondence
    @assert corrs <= 1
    # check if already added, add and set flag
    if corrs == 1 && mdt < Millisecond(100)
      rangeIdx = ind[2]
      if !dashboard[:rangesBuffer][rangeIdx][3][1]
        bera = Pose2Point2Range(AliasingScalarSampler(dashboard[:rangesBuffer][rangeIdx][2][:,1],dashboard[:rangesBuffer][rangeIdx][2][:,2], SNRfloor=dashboard[:SNRfloor]))
        addFactor!(dfg, [lastPose; :l1], bera, solvable=0, autoinit=false)
        fSym = lastPose*"l1f1"
        setTimestamp!(getFactor(dfg, fSym), msgtime)
        dashboard[:rangesBuffer][rangeIdx][3][1] = true
        # check that ranges are not lost owing to clock drift
        deltaClk = lastT - dashboard[:rangesBuffer][rangeIdx][1]
        @assert abs(deltaClk) < Millisecond(100)

        # set the factors as solvable (landmark if not there yet too)
        addLm = getSolvable(dfg, :l1) == 1 ? Symbol[] : [:l1; :l1f1]
        put!(dashboard[:solvables], [fSym;addLm])
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
    # update odo time
    dashboard[:odoTime] = odoT

    # get factor to next pose
    nPose = nextPose(dashboard[:lastPose])
    @info "pose_hdlr, adding new pose $nPose, at $odoT"

    rttLast = dashboard[:rttMpp][dashboard[:lastPose]]
    # add new variable and factor to the graph
    fctsym = duplicateToStandardFactorVariable(Pose2Pose2,
                                               rttLast,
                                               dfg,
                                               dashboard[:lastPose],
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
    nRtt = Symbol(string("drt_",string(nPose)[2:end]))
    addVariable!(dfg, nRtt, Pose2, solvable=0)
    drec = MutablePose2Pose2Gaussian(MvNormal(zeros(3), Matrix{Float64}(LinearAlgebra.I, 3,3)))
    addFactor!(dfg, [nPose; nRtt], drec, solvable=0, autoinit=false)
    dashboard[:rttMpp][nPose] = drec
    put!(dashboard[:solvables], [nPose; fctsym])
    dashboard[:poseStride] += 1

    # separate check for LCMLog handling
    # if dashboard[:solveInProgress] == SSMSolving
    #   dashboard[:canTakePoses] = HSMOverlapHandling
    # elseif dashboard[:solveInProgress] == SSMReady || dashboard[:solveInProgress] == SSMConsumingSolvables
    #   dashboard[:canTakePoses] = HSMHandling
    # end
    if dashboard[:doDelay] && 10 <= dashboard[:poseStride]
      # how far to escalate -- :canTakePoses is de-escalated by solver in manageSolveTree()
      if dashboard[:canTakePoses] == HSMHandling
        dashboard[:canTakePoses] = HSMOverlapHandling
      elseif dashboard[:canTakePoses] == HSMOverlapHandling
        dashboard[:canTakePoses] = HSMBlocking
      end
    end
  end

  nothing
end

#
