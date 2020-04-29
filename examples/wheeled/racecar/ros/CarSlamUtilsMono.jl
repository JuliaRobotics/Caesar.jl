## requirements in this file

using LinearAlgebra


## monocular camera on car tools

include(joinpath(@__DIR__, "CarSlamUtilsCommon.jl"))


##

# find joystick command segment
# get all inputs up to this next pose
function whichJoystickValues(fec, fromT::DateTime, upToTime::DateTime)
  # upToTime = msgTime # getTimestamp(getVariable(fec.slam.dfg,addHist[end]))
  cmdTimes = (x->fec.synchronizer.cmdVal[x][2]).(1:length(fec.synchronizer.cmdVal)) .|> nanosecond2datetime
  addHist = getAddHistory(fec.slam.dfg)
  filter!(x->occursin(r"x\d",string(x)), addHist)
  mask = fromT .<= cmdTimes .< upToTime

  idxes = (1:length(fec.synchronizer.cmdVal))[mask]
  # get the actual times from the buffer
  joyValsTuple = fec.synchronizer.cmdVal[idxes]
  # @show typeof(joyValsTuple)
  # @show keys(joyValsTuple[1][3])
  # @show typeof(joyValsTuple[1][3])
  # @show joyValsTuple[1][3].axis
  # (x->x[3][:axis][[2;4]]).(joyValsTuple)
  # (x->x[3]).(joyValsTuple)

  # Tuple{Int,Int,Dict{Symbol, Any}}
  joyValsTuple
end

# for FluxModelsPose2Pose2
# ::AbstractArray{<:Real}
@everywhere function interpToOutx4(indata::AbstractMatrix{<:Real}; outlen=25)
  #
  len = size(indata,1)
  if 0 == len
    return zeros(outlen,4)
  end

  # assume evenly spaced
  idx = 1:len
  # tsLcl = range(indata[1,1],indata[end,1],length=outlen)
  iThrtTemp = DataInterpolations.LinearInterpolation(indata[:,1], idx)
  iSterTemp = DataInterpolations.LinearInterpolation(indata[:,2], idx)
  iVelxTemp = DataInterpolations.LinearInterpolation(indata[:,3], idx)
  iVelyTemp = DataInterpolations.LinearInterpolation(indata[:,4], idx)
  newArr = Matrix{Float64}(undef, outlen, 4)
  lookup = range(1,len, length=outlen)
  for i in 1:outlen
    lid = lookup[i]
    newArr[i, 1] = iThrtTemp(lid)
    newArr[i, 2] = iSterTemp(lid)
    newArr[i, 3] = iVelxTemp(lid)
    newArr[i, 4] = iVelyTemp(lid)
  end
  # currently have no velocity values
  return newArr
end

@everywhere function interpToOutx2(indata::AbstractMatrix{<:Real}; outlen=25)
  #
  @show typeof(indata)
  @show size(indata)
  len = size(indata,1)
  if 0 == len
    return zeros(outlen,2)
  end

  # assume evenly spaced
  idx = 1:len
  # tsLcl = range(indata[1,1],indata[end,1],length=outlen)
  iThrtTemp = DataInterpolations.LinearInterpolation(indata[:,1], idx)
  iSterTemp = DataInterpolations.LinearInterpolation(indata[:,2], idx)
  newArr = Matrix{Float64}(undef, outlen, 2)
  lookup = range(1,len, length=outlen)
  for i in 1:outlen
    lid = lookup[i]
    newArr[i, 1] = iThrtTemp(lid)
    newArr[i, 2] = iSterTemp(lid)
  end
  # currently have no velocity values
  return newArr
end

@everywhere function interpToOutx2(indata::Vector{<:AbstractVector{<:Real}}; outlen=25)
  if length(indata) == 1
    @warn "interpToOutx2 indata is too short, returning zeros"
    return zeros(25,2)
  end
  arr = zeros(length(indata),2)
  for i in 1:length(indata), j in 1:2
    arr[i, j] = indata[i][j]
  end
  interpToOutx2(arr, outlen=outlen)
end

@everywhere function interpToOutx2(indata::Vector{Any}; outlen=25)
  if length(indata) == 0
    return zeros(25,2)
  else
    @show indata
    error("interpToOutx2(indata::Vector{Any} does not know how to work with (see above)")
  end
end

# @everywhere function interpTo25x4Old(lclJD)
#   @warn "Unsure of interpTo25x4Old is doing the right thing (this is the old version)"
#   if 0 == size(lclJD,1)
#     return [zeros(4) for i in 1:25]
#   end
#
#   tsLcl = range(lclJD[1,1],lclJD[end,1],length=25)
#   intrTrTemp = DataInterpolations.LinearInterpolation(lclJD[:,2],lclJD[:,1])
#   intrStTemp = DataInterpolations.LinearInterpolation(lclJD[:,3],lclJD[:,1])
#   newVec = Vector{Vector{Float64}}()
#   for tsL in tsLcl
#     newVal = zeros(4)
#     newVal[1] = intrTrTemp(tsL)
#     newVal[2] = intrStTemp(tsL)
#     push!(newVec, newVal)
#   end
#   # currently have no velocity values
#   return newVec
# end

# Is this a general requirement?
function updateSLAMMono!(fec::FrontEndContainer,
                         WEIRDOFFSET::Dict,
                         syncList::Vector{Symbol}=fec.synchronizer.syncList;
                         useFluxModels::Bool=false,
                          allModels=nothing,
                          naiveFrac::Float64=0.6)
  #
  syncz = fec.synchronizer
  idxL,idxO = findSyncLatestIdx(syncz, syncList=[:leftFwdCam;:camOdo], weirdOffset=WEIRDOFFSET)
  if idxL == 0 || idxO == 0
    return nothing
  end
  # already have a counter
  fec.slam.frameCounter = syncz.camOdo[idxO][1]

  # calculate delta odo since last pose
  zTi = fec.slam.helpers.lastPoseOdomBuffer
  zT = syncz.camOdo[idxO][3]
  iT = zTi\zT
  # x-fwd, y-right, z-up
  psi = convert(Euler, iT.R).Y  # something not right here?
  DX = [iT.t[1];iT.t[2]; psi]

  # Tigger a new pose or skip out
  if !(0 < abs.(DX[3]) - parsed_args["pose_trigger_rotate"] ||
       0 < norm(DX[1:2]) - parsed_args["pose_trigger_distance"])
    # havent travelled far enough yet
    return nothing
  end
  @show round.(DX, digits=3)
  # psi = convert(Euler, zT.R).Y - convert(Euler, zTi.R).Y |> TU.wrapRad
  # @show convert(Euler, iT.R).Y  # something not right here?

  # reset interpose odometry counter for outside global odo integration process
  fec.slam.helpers.lastPoseOdomBuffer = syncz.camOdo[idxO][3] |> deepcopy

  # add new pose
  prevpose = Symbol("x$(fec.slam.poseCount)")
  fec.slam.poseCount += 1
  newpose = Symbol("x$(fec.slam.poseCount)")
  # take timestamp from image msg
  imgTime = nanosecond2datetime(syncz.leftFwdCam[idxL][2])
  addVariable!(fec.slam.dfg, newpose, Pose2, timestamp=imgTime)

  # previous pose time
  prevPoseT = getTimestamp(getVariable(fec.slam.dfg,prevpose))

  # find joystick values required
  joyVals = whichJoystickValues(fec, prevPoseT, imgTime)
  ## lets store the values while we're here
  addDataEntry!( fec.slam.dfg, prevpose, fec.datastore, :JOYSTICK_CMD_VALS, "application/json", Vector{UInt8}(JSON2.write( joyVals ))  )

  baselineOdo = MvNormal(DX, diagm([0.05; 0.05; 0.3].^2))
  pp = if useFluxModels
    # convert to NN required format
    # joyVelVal = catJoyVelData(joyVals, velVal)
    # interpolate joystick values to correct length
    joyAs25x2 = ( x->[ x[3][:axis][2]; x[3][:axis][2] ] ).(joyVals)
      @show typeof(joyAs25x2), size(joyAs25x2)
    joyVals25x2 = interpToOutx2( joyAs25x2 )
    # @show size(joyVals25x2)
    joyVals25 = hcat(joyVals25x2, zeros(25,2))
    # build the factor
    FluxModelsPose2Pose2(allModels, joyVals25, baselineOdo, naiveFrac)
  else
    # TODO add covariance later
    Pose2Pose2(baselineOdo)
  end
  # take timestamp from image msg
  addFactor!(fec.slam.dfg, [prevpose;newpose], pp,
             timestamp=nanosecond2datetime(syncz.camOdo[idxO][2]) )

  # variables that can be initialized / solved
  put!(fec.slam.solveSettings.solvables, [newpose; ls(fec.slam.dfg,newpose)[1]])

  # add any potential landmarks and factors
  tagsL = detector(syncz.leftFwdCam[idxL][3] |> collect)
  # tagsR = detector(syncHdlr[:right][idxR][2])

  @show tagsL |> length

  for tag in tagsL
    @show lmid = Symbol("l$(tag.id)")
    !exists(slam.dfg, lmid) && addVariable!(slam.dfg, lmid, Pose2)
    # pose = homographytopose(tag.H, fx, fy, cx, cy, taglength = 160.)
    pose, err1 = tagOrthogonalIteration(tag, fx, fy, cx, cy, taglength = 0.25)
    cTt = LinearMap(pose[1:3, 1:3])∘Translation((pose[1:3,4])...)
    bRc = Quat(1/sqrt(2),0,0,-1/sqrt(2))*Quat(1/sqrt(2),-1/sqrt(2),0,0)
    bTt = LinearMap(bRc) ∘ cTt
    # wTb = LinearMap(zT.R.R) ∘ Translation(zT.t...)
    # wTt = wTb ∘ bTt

    ld = LinearAlgebra.cross(bTt.linear*[0;0;1], [0;0;1])
    theta = atan(ld[2],ld[1])
    Dtag = [bTt.translation[1:2,];theta]
    # println("$(tag.id), $(ld[3]), $(round.(Dtag, digits=3))")
    p2l2 = Pose2Pose2(MvNormal(Dtag,diagm([0.1;0.1;0.1].^2)))
    addFactor!(slam.dfg, [newpose;lmid], p2l2, timestamp=imgTime)

    # FIXME should be in FEC
    if parsed_args["vis3d"]
      setobject!(vis[:tags][Symbol("tag_$(tag.id)")][:triad], Triad(0.2))
      tagobj = HyperRectangle(Vec(0.,0,0), Vec(0.3,0.3,0.03))
      setobject!(vis[:tags][Symbol("tag_$(tag.id)")][:plate], tagobj)
      settransform!(vis[:tags][Symbol("tag_$(tag.id)")], bTt) # bTt
    end

    # setobject!(vis[:poses][Symbol(newpose)], Triad(0.5))
    # settransform!(vis[:poses][Symbol(newpose)], wTb)

  end

  # check whether a solve should be trigger
  checkSolveStrideTrigger!(fec.slam)

  nothing
end




function drawLatestImage(fec::FrontEndContainer; syncList=[:leftFwdCam;])
  # get synced images
  idxL = findSyncLatestIdx(fec.synchronizer, weirdOffset=WEIRDOFFSET, syncList=syncList )

  # @info "drawLatestImagePair, $idxL, $idxR, $idxO"
  if idxL == 0
    return nothing
  end

  # Do apriltag detection
  tagsL = detector(fec.synchronizer.leftFwdCam[idxL][3] |> collect)

  # @show poses = (T->homographytopose(T.H, fx, fy, cx, cy, taglength = 160.)).(tagsL)
  imgLt = showImage(fec.synchronizer.leftFwdCam[idxL][3], tagsL, K)

  # imgL = syncImgs[:left][idxL][2] .|> Gray
  # imgR = syncImgs[:right][idxR][2] .|> Gray
  # featuresL = Keypoints(fastcorners(imgL, 12, 0.25))
  # featuresR = Features(fastcorners(imgR, 12, 0.25))
  # offsetx = CartesianIndex(0, 5)
  # offsety = CartesianIndex(5, 0)
  # map(m -> draw!(imgLt, LineSegment(m - offsetx, m + offsetx)), featuresL)
  # map(m -> draw!(imgLt, LineSegment(m - offsety, m + offsety)), featuresL)

  # draw both
  if parsed_args["imshow"]
    imshow(canvases[1,1], imgLt)
    Gtk.showall(gui["window"])
  end
  nothing
end


##  MessageHandler


function leftImgHdlr(msgdata, fec::FrontEndContainer)
  # @show "leftImgHdlr", msgdata[2].header.seq
  # @show "leftImgHdlr"
  mtms = getROSPyMsgTimestamp(msgdata[end])
  nsT = Int64(datetime2unix(mtms[1]))*1000_000_000 + mtms[2]

  leftdata = take!(IOBuffer(msgdata[2].data))

  # more direct image from ROS and zed
  nvi = normedview(leftdata)
  # im3 = last(fec.synchronizer.leftFwdCam)[2]
  arr = reshape(nvi, 3, 672,376) # im3
  pv = PermutedDimsArray(arr, (1,3,2))
  pvn = similar(pv)
  pvn[3,:,:] .= pv[1,:,:]
  pvn[2,:,:] .= pv[2,:,:]
  pvn[1,:,:] .= pv[3,:,:]
  ldImg = colorview(RGB, pvn)

  # for compressed images
  # ldImg = ImageMagick.load_(leftdata)
  push!(fec.synchronizer.leftFwdCam, (msgdata[2].header.seq, nsT, ldImg) )
  # img = last(SyncImages[:left])
  nothing
end


function odomHdlr(msgdata, fec::FrontEndContainer, WO, allModels=nothing)
  # @show "odomHdlr", msgdata[2].header.seq
  # @show "odomHdlr"
  mtms = getROSPyMsgTimestamp(msgdata[end])
  nsT = Int64(datetime2unix(mtms[1]))*1000_000_000 + mtms[2]

  pos = msgdata[2].pose.pose.position
  quat = msgdata[2].pose.pose.orientation

  zPi = [pos.x;pos.y;pos.z]
  zQi = TU.Quaternion(quat.w,[quat.x;quat.y;quat.z])

  zTi = SE3(zPi, zQi)
  push!(fec.synchronizer.camOdo, (msgdata[2].header.seq, nsT, zTi) )

  # show leftFwdCam image
  drawLatestImage(fec, syncList=[:leftFwdCam;])

  # update the factor graph object as required
  if allModels == nothing
    updateSLAMMono!(fec, WO, useFluxModels=false )
  else
    updateSLAMMono!(  fec, WO, useFluxModels=true, allModels=allModels, naiveFrac=parsed_args["naive_frac"]  )
  end

  nothing
end


function joystickHdlr(msgdata, fec::FrontEndContainer)
  # @show "joystickHdlr", msgdata[2].header.seq
  mtms = getROSPyMsgTimestamp(msgdata[end])
  nsT = Int64(datetime2unix(mtms[1]))*1000_000_000 + mtms[2]

  data = Dict(:axis => msgdata[2].axes,
              :buttons => msgdata[2].buttons)
  # @show data
  push!(fec.synchronizer.cmdVal, (msgdata[2].header.seq, nsT, data) )

  #
end



#
