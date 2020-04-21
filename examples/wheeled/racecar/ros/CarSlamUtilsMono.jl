# monocular camera on car tools

include(joinpath(@__DIR__, "CarSlamUtilsCommon.jl"))


##

# Is this a general requirement?
function updateSLAMMono!(fec::FrontEndContainer,
                         WEIRDOFFSET::Dict,
                         syncList::Vector{Symbol}=fec.synchronizer.syncList)
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
  zT = syncz.camOdo[idxO][2]
  iT = zTi\zT
  # x-fwd, y-right, z-up
  psi = convert(Euler, iT.R).Y  # something not right here?
  DX = [iT.t[1];iT.t[2]; psi]
  if !(0 < abs.(DX[3]) - pi/4 || 0 < norm(DX[1:2]) - 0.5)
    # havent travelled far enough yet
    return nothing
  end
  @show round.(DX, digits=3)
  # psi = convert(Euler, zT.R).Y - convert(Euler, zTi.R).Y |> TU.wrapRad
  # @show convert(Euler, iT.R).Y  # something not right here?

  # reset interpose odometry counter for outside global odo integration process
  fec.slam.helpers.lastPoseOdomBuffer = syncz.camOdo[idxO][2] |> deepcopy

  # add new pose
  prevpose = Symbol("x$(fec.slam.poseCount)")
  fec.slam.poseCount += 1
  newpose = Symbol("x$(fec.slam.poseCount)")
  addVariable!(fec.slam.dfg, newpose, Pose2)

  # TODO add covariance later
  pp = Pose2Pose2(MvNormal(DX, diagm([0.05; 0.05; 0.3].^2)))
  # pp = Pose2Pose2(MvNormal([0.1;0.0;0.0], diagm([0.4; 0.2; 0.3].^2)))
  addFactor!(fec.slam.dfg, [prevpose;newpose], pp)

  # variables that can be initialized / solved
  put!(fec.slam.solveSettings.solvables, [newpose; ls(fec.slam.dfg,newpose)[1]])

  # add any potential landmarks and factors
  tagsL = detector(syncz.leftFwdCam[idxL][2] |> collect)
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

    ld = cross(bTt.linear*[0;0;1], [0;0;1])
    theta = atan(ld[2],ld[1])
    Dtag = [bTt.translation[1:2,];theta]
    # println("$(tag.id), $(ld[3]), $(round.(Dtag, digits=3))")
    p2l2 = Pose2Pose2(MvNormal(Dtag,diagm([0.1;0.1;0.1].^2)))
    addFactor!(slam.dfg, [newpose;lmid], p2l2)

    setobject!(vis[:tags][Symbol("tag_$(tag.id)")][:triad], Triad(0.2))
    tagobj = HyperRectangle(Vec(0.,0,0), Vec(0.3,0.3,0.03))
    setobject!(vis[:tags][Symbol("tag_$(tag.id)")][:plate], tagobj)
    settransform!(vis[:tags][Symbol("tag_$(tag.id)")], bTt) # bTt

    # setobject!(vis[:poses][Symbol(newpose)], Triad(0.5))
    # settransform!(vis[:poses][Symbol(newpose)], wTb)
  end

  # check whether a solve should be trigger
  checkSolveStrideTrigger!(fec.slam)

  nothing
end




function drawLatestImage(fec::FrontEndContainer; syncList=[:leftFwdCam;])
  # get synced images
  @show idxL = findSyncLatestIdx(fec.synchronizer, weirdOffset=WEIRDOFFSET, syncList=syncList )

  # @info "drawLatestImagePair, $idxL, $idxR, $idxO"
  if idxL == 0
    return nothing
  end

  # Do apriltag detection
  tagsL = detector(fec.synchronizer.leftFwdCam[idxL][2] |> collect)

  # @show poses = (T->homographytopose(T.H, fx, fy, cx, cy, taglength = 160.)).(tagsL)
  imgLt = showImage(fec.synchronizer.leftFwdCam[idxL][2], tagsL, K)

  # imgL = syncImgs[:left][idxL][2] .|> Gray
  # imgR = syncImgs[:right][idxR][2] .|> Gray
  # featuresL = Keypoints(fastcorners(imgL, 12, 0.25))
  # featuresR = Features(fastcorners(imgR, 12, 0.25))
  # offsetx = CartesianIndex(0, 5)
  # offsety = CartesianIndex(5, 0)
  # map(m -> draw!(imgLt, LineSegment(m - offsetx, m + offsetx)), featuresL)
  # map(m -> draw!(imgLt, LineSegment(m - offsety, m + offsety)), featuresL)

  # draw both
  imshow(canvases[1,1], imgLt)

  Gtk.showall(gui["window"])
  nothing
end


##  MessageHandler


function leftImgHdlr(msgdata, fec::FrontEndContainer)
  # @show "leftImgHdlr", msgdata[2].header.seq
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
  push!(fec.synchronizer.leftFwdCam, (msgdata[2].header.seq, ldImg) )
  # img = last(SyncImages[:left])
  nothing
end


function odomHdlr(msgdata, fec::FrontEndContainer, WO)
  # @show "odomHdlr", msgdata[2].header.seq

  pos = msgdata[2].pose.pose.position
  quat = msgdata[2].pose.pose.orientation

  zPi = [pos.x;pos.y;pos.z]
  zQi = TU.Quaternion(quat.w,[quat.x;quat.y;quat.z])

  zTi = SE3(zPi, zQi)
  push!(fec.synchronizer.camOdo, (msgdata[2].header.seq, zTi) )

  drawLatestImage(fec, syncList=[:leftFwdCam;])
  updateSLAMMono!(fec, WO)

  nothing
end


function joystickHdlr(msgdata, fec::FrontEndContainer)
  # @show "joystickHdlr", msgdata[2].header.seq
  data = Dict(:axis => msgdata[2].axes,
              :buttons => msgdata[2].buttons)
  # @show data
  push!(fec.synchronizer.cmdVal, (msgdata[2].header.seq, data) )

  #
end



#
