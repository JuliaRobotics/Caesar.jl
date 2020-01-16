# all the message handlers for AUV example


## middleware handler (callback) functions

function lbl_hdlr(channel::String, msgdata::pose_t, dfg::AbstractDFG, dashboard::Dict, lblDict::Dict, lbllog)
  msgtime = unix2datetime(msgdata.utime*1e-6)
  dashboard[:lastMsgTime] = msgtime

  # TODO -- use buffer instead (legacy)
  if length(lblDict) == 0
    lblDict[dashboard[:lastMsgTime]] = (msgdata.pos[1:2] |> deepcopy)
  end

  # add to the ciruclar buffer
  push!( dashboard[:lblBuffer], (msgtime, (msgdata.pos[1:2] |> deepcopy), Bool[false;]) )

  # also record in log for later printing
  println(lbllog, "$msgtime, $(msgdata.pos[1]),$(msgdata.pos[2])")

  # check if any lbl can be added as prior to factor graph
  for lbl in dashboard[:lblBuffer]
    # might have been added in the past
    lbl[3][1] ? continue : nothing
    # find nearest in time
    corr = findVariableNearTimestamp(dfg, lbl[1], r"x\d")  #tags=[:POSE;])
    # is there a match and its real close in time (assume 50Hz max rate)
    if 1 == length(corr) && abs(corr[1][2]) < Millisecond(10)
      # set the reference data in that variable
      setVariableRefence!(dfg, corr[1][1][1], reshape(lbl[2],:,1), refKey=:lbl)
      lbl[3][1] = true
    end
  end
  nothing
end

function mag_hdlr(channel::String, msgdata::pose_t, dfg::AbstractDFG, dashboard::Dict, magDict::Dict)
  msgtime = unix2datetime(msgdata.utime*1e-6)
  dashboard[:lastMsgTime] = msgtime
  ea = TU.convert(Euler, Quaternion(msgdata.orientation[1],msgdata.orientation[2:4]) )

  # add first element only
  # TODO use buffer instead (legacy)
  if length(magDict) == 0
    magDict[dashboard[:lastMsgTime]] = ea.Y
  end

  push!( dashboard[:magBuffer], (msgtime, ea.Y, Bool[false;]) )

  # check if any mag can be added as prior to factor graph
  for mag in dashboard[:magBuffer]
    # might have been added in the past
    mag[3][1] ? continue : nothing
    corr = findVariableNearTimestamp(dfg, mag[1], r"x\d")  #tags=[:POSE;])
    # is there a match and its real close in time (assume 50Hz max rate)
    if 1 == length(corr) && abs(corr[1][2]) < Millisecond(10)
      # double check that the variable does not already have a mag prior on it
      if !hasTagsNeighbors(dfg, corr[1][1][1], [:MAGNETOMETER;])
        # add the mag measurement as a prior
        @show "add mag"
        magPrior = PartialPriorYawPose2(Normal(mag[2], dashboard[:magNoise]))
        addFactor!(dfg, corr[1][1][1:1], magPrior, autoinit=false, solvable=1, labels=[:MAGNETOMETER;])
        # set flag that this has already been added to the factor graph
        mag[3][1] = true
      end
    end
  end

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
    lastT = getTimestamp(getVariable(dfg, lastPose))
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

function pose_hdlr(channel::String,
                   msgdata::pose_t,
                   dfg::AbstractDFG,
                   dashboard::Dict,
                   timinglog,
                   drtlog,
                   drolog,
                   dirodolog,
                   rawodolog,
                   alltht )
  #
  t0 = time_ns()
  odoT = unix2datetime(msgdata.utime*1e-6)
  dashboard[:lastMsgTime] = odoT # unix2datetime(msgdata.utime*1e-6)
  # "accumulateDiscreteLocalFrame! on all RTTs" |> println
  for (vsym, drt) in dashboard[:drtMpp]
    for symFct in intersect(ls(dfg, MutablePose2Pose2Gaussian), ls(dfg,vsym))
      drtFct = getFactorType(dfg, symFct)
      accumulateDiscreteLocalFrame!(drtFct, msgdata.pos[1:3], dashboard[:Qc_odo], 1.0, Phik=SE2(msgdata.pos[1:3])) # drt
      println(alltht, "$odoT, $(length(dashboard[:drtMpp])), $vsym, $(symFct), $(msgdata.pos[1]), $(msgdata.pos[2]), $(msgdata.pos[3]), $(drtFct.Zij.μ[1]), $(drtFct.Zij.μ[2]), $(drtFct.Zij.μ[3])")
    end
  end
  dt_acc = (time_ns()-t0)/1e9

  # get message time

  # write the latest DRT solution to file
  drtFnc = string(dashboard[:drtCurrent][1], dashboard[:drtCurrent][2], "f1") |> Symbol
  val = accumulateFactorMeans(dfg, [drtFnc;])
  drtFncMu = getFactorType(dfg, drtFnc).Zij.μ
  println(drtlog, "$odoT, $(drtFnc), $(dashboard[:lastPose]), $(val[1]), $(val[2]), $(val[3]), $(msgdata.pos[1]), $(msgdata.pos[2]), $(msgdata.pos[3]), $(collect(keys(dashboard[:drtMpp]))), $(drtFncMu)")
  drval = accumulateFactorMeans(dfg, [:x0drt_0f1;])
  println(drolog, "$odoT, $(dashboard[:lastPose]), $(drval[1]), $(drval[2]), $(drval[3])")
  dt_drt = (time_ns()-t0)/1e9

  # check if a new pose and factor must be added?
  if (dashboard[:odoTime] + dashboard[:poseRate]) <= odoT
    # update odo time
    dashboard[:odoTime] = odoT

    # get factor to next pose
    nPose = nextPose(dashboard[:lastPose])
    @info "pose_hdlr, adding new pose $nPose, at $odoT.  DRTs $(collect(keys(dashboard[:drtMpp]))), drtCurrent=$(dashboard[:drtCurrent])"

    # get drt from last pose
    lastPoseDrt = intersect(ls(dfg, dashboard[:lastPose]), ls(dfg, MutablePose2Pose2Gaussian))[1]
    drtLast = getFactorType(dfg, lastPoseDrt)
    # drtLast = dashboard[:drtMpp][dashboard[:lastPose]]

    # add new variable and factor to the graph
    fctsym = duplicateToStandardFactorVariable(Pose2Pose2,
                                               drtLast,
                                               dfg,
                                               dashboard[:lastPose],
                                               nPose,
                                               solvable=0,
                                               autoinit=false,
                                               cov=dashboard[:odoCov]  )
    #
    # set timestamp to msg times
    setTimestamp!(getVariable(dfg, nPose), odoT)
    # delete drt unless used by real time prediction
    # TODO assuming stride ends on a 0
    poseStrideTail = sortDFG(ls(dfg, r"x\d+9\b|x9\b", solvable=0))
    poseStrideTail = 3 < length(poseStrideTail) ? poseStrideTail[end-3:end] : poseStrideTail
    for vkey in keys(dashboard[:drtMpp])
      if vkey != dashboard[:lastPose] && vkey != :x0 && !(vkey in poseStrideTail) # dashboard[:drtCurrent][1]
      # if dashboard[:lastPose] != :x0 && !(dashboard[:lastPose] in poseStrideTail) # dashboard[:drtCurrent][1]
        delete!(dashboard[:drtMpp], vkey)
        # delete!(dashboard[:drtMpp], dashboard[:lastPose])
      end
    end

    # update last pose to new pose
    dashboard[:lastPose] = nPose

    # create new drt on new variable
    nDrt = Symbol(string("drt_",string(nPose)[2:end]))
    addVariable!(dfg, nDrt, Pose2, solvable=0)
    drec1 = MutablePose2Pose2Gaussian(MvNormal(zeros(3), Matrix{Float64}(LinearAlgebra.I, 3,3)))
    addFactor!(dfg, [nPose; nDrt], drec1, solvable=0, autoinit=false)
    dashboard[:drtMpp][nPose] = drec1
    put!(dashboard[:solvables], [nPose; fctsym])
    dashboard[:poseStride] += 1

    if dashboard[:doDelay] && 10 <= dashboard[:poseStride]
      # how far to escalate -- :canTakePoses is de-escalated by solver in manageSolveTree()
      if dashboard[:canTakePoses] == HSMHandling
        dashboard[:canTakePoses] = HSMOverlapHandling
      elseif dashboard[:canTakePoses] == HSMOverlapHandling
        dashboard[:canTakePoses] = HSMBlocking
      end
      @info "new solve token $nPose"
      put!(dashboard[:poseSolveToken], nPose)
      dashboard[:poseStride] = 0
    end

    # also store current factor graph if part of plotting request
    if dashboard[:savePlotting]
      dfg_ = deepcopy(dfg)
      @spawn saveDFG(dfg_, joinpath(getLogPath(dfg_), "fg_$nPose"))
    end
  end
  dt_pose = (time_ns()-t0)/1e9

  ## Separate odometry check
  # mutable pose2pose2 factor holding odometry only reference
  # mpp = dashboard[:drtOdoRef][3]
  # accumulate odo
  accumulateDiscreteLocalFrame!(getFactorType(dfg, :x0drt_reff1), msgdata.pos[1:3], dashboard[:Qc_odo], 1.0, Phik=SE2(msgdata.pos[1:3]))
  # write result to file
  drval = accumulateFactorMeans(dfg, [:x0drt_reff1;])
  println(dirodolog, "$odoT, $(dashboard[:lastPose]), $(drval[1]), $(drval[2]), $(drval[3])")
  println(rawodolog, "$odoT, $(dashboard[:lastPose]), $(msgdata.pos[1]), $(msgdata.pos[2]), $(msgdata.pos[3])")

  dt_total = (time_ns()-t0)/1e9
  println(timinglog, "$odoT, $(dashboard[:lastPose]), $t0, $dt_acc, $dt_drt, $dt_pose, $dt_total")

  nothing
end


#
