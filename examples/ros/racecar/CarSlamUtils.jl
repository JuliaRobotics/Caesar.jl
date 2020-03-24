


## Working memory

SyncImages = Dict{Symbol, CircularBuffer{Tuple{Int, Any}}}()
SyncImages[:left] = CircularBuffer{Tuple{Int, Any}}(30) # ImgType
SyncImages[:right] = CircularBuffer{Tuple{Int, Any}}(30)
SyncImages[:zedodo] = CircularBuffer{Tuple{Int, Any}}(30)

delete!(vis[:tags])
delete!(vis[:poses])

## User functions

function showImage(image, tags, K)
    # Convert image to RGB
    imageCol = RGB.(image)
    #traw color box on tag corners
    foreach(tag->drawTagBox!(imageCol, tag, width = 2, drawReticle = false), tags)
    foreach(tag->drawTagAxes!(imageCol,tag, K), tags)
    imageCol
end

function getSyncLatestPair(syncHdlrs::Dict; weirdOffset::Dict=Dict())::Tuple
  # get all sequence numbers
  ks = keys(syncHdlrs) |> collect
  len = length(ks)
  LEN = Vector{Int}(undef, len)
  for idx in 1:len
    LEN[idx] = syncHdlrs[ks[idx]] |> length
    if LEN[idx] == 0
      return (NTuple{len, Int}(zeros(len)))
    end
  end
  # lenL = length(syncHdlrs[:left])
  # lenR = length(syncHdlrs[:right])
  # if lenL == 0 || lenR == 0
  #   return (0,0)
  # end
  SEQ = Vector{Vector{Int}}(undef, len)
  for idx in 1:len
    SEQ[idx] = (x->syncHdlrs[ks[idx]][x][1]).(1:LEN[idx])
    if haskey(weirdOffset, ks[idx])
      SEQ[idx] .+= weirdOffset[ks[idx]]
    end
  end
  # seqL = (x->syncHdlrs[:left][x][1]).(1:lenL)
  # seqR = (x->syncHdlrs[:right][x][1]).(1:lenR)
  # find greatest common number

  seqLR = intersect(SEQ...)
  # seqLR = intersect(seqL, seqR)
  if 0 == length(seqLR)
    return NTuple{len,Int}(zeros(len))
  end
  seq = maximum(seqLR)
  IDX = Vector{Int}(undef, len)

  # idxL = findfirst(x->x==seq, seqL)
  # idxR = findfirst(x->x==seq, seqR)
  return NTuple{len, Int}( (y->findfirst(x->x==seq, y)).(SEQ) )
  # (idxL, idxR)
end

function drawLatestImagePair(syncImgs)
  # get synced images
  idxL, idxR, idxO = getSyncLatestPair(syncImgs, weirdOffset=WEIRDOFFSET)

  # @info "drawLatestImagePair, $idxL, $idxR, $idxO"
  if idxL == 0 || idxR == 0
    return nothing
  end

  # Do apriltag detection
  tagsL = detector(syncImgs[:left][idxL][2])
  tagsR = detector(syncImgs[:right][idxR][2])

  # @show poses = (T->homographytopose(T.H, fx, fy, cx, cy, taglength = 160.)).(tagsL)
  imgLt = showImage(syncImgs[:left][idxL][2], tagsL, K)
  imgRt = showImage(syncImgs[:right][idxR][2], tagsR, K)

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
  imshow(canvases[1,2], imgRt)
  # imshow(canvases[1,1], syncImgs[:left][idxL][2])
  # imshow(canvases[1,2], syncImgs[:right][idxR][2])

  Gtk.showall(gui["window"])
  nothing
end

##  SLAM Functions


mutable struct SLAMCommonHelper
  lastPoseOdomBuffer::SE3 # common helper
  SLAMCommonHelper(lpo::SE3=SE3(0)) = new(lpo)
end

mutable struct SLAMWrapperLocal{G <: AbstractDFG} <: AbstractSLAM
  dfg::G
  poseCount::Int
  frameCounter::Int
  poseStride::Int # pose every frameStride (naive pose trigger)
  helpers::SLAMCommonHelper
  solveSettings::ManageSolveSettings
end

SLAMWrapperLocal(;dfg::G=initfg(),
                  poseCount::Int=0,
                  frameCounter::Int=0,
                  poseStride::Int=10,
                  helpers::SLAMCommonHelper=SLAMCommonHelper(),
                  solveSettings::ManageSolveSettings=ManageSolveSettings() ) where {G <: AbstractDFG}= SLAMWrapperLocal{G}(dfg,
                      poseCount, frameCounter, poseStride, helpers, solveSettings)
#






function updateSLAM!(slamw::SLAMWrapperLocal, syncHdlr)
  idxL, idxR, idxO = getSyncLatestPair(syncHdlr, weirdOffset=WEIRDOFFSET)
  if idxL == 0
    return nothing
  end
  # already have a counter
  slamw.frameCounter = syncHdlr[:zedodo][idxO][1]
  # bad traffic, i.e. useless rule
  # if slamw.frameCounter % slamw.poseStride != 0
  #   return nothing
  # end

  # calculate delta odo since last pose
  zTi = slamw.helpers.lastPoseOdomBuffer
  zT = syncHdlr[:zedodo][idxO][2]
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
  slamw.helpers.lastPoseOdomBuffer = syncHdlr[:zedodo][idxO][2] |> deepcopy

  # add new pose
  prevpose = Symbol("x$(slamw.poseCount)")
  slamw.poseCount += 1
  newpose = Symbol("x$(slamw.poseCount)")
  addVariable!(slamw.dfg, newpose, Pose2)

  # TODO add covariance later
  pp = Pose2Pose2(MvNormal(DX, diagm([0.05; 0.05; 0.3].^2)))
  # pp = Pose2Pose2(MvNormal([0.1;0.0;0.0], diagm([0.4; 0.2; 0.3].^2)))
  addFactor!(slamw.dfg, [prevpose;newpose], pp)

  # variables that can be initialized / solved
  put!(slamw.solveSettings.solvables, [newpose; ls(slamw.dfg,newpose)[1]])

  # add any potential landmarks and factors
  tagsL = detector(syncHdlr[:left][idxL][2])
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

  # FIXME
  if slamw.poseCount % slamw.solveSettings.solveStride == 0
    @info "trigger a solve, $newpose, $(length(slamw.solveSettings.poseSolveToken.data)) ====================================="
    put!(slamw.solveSettings.poseSolveToken, newpose)
    @info "done put of $newpose"
    # also update DRT containers
  end

  nothing
end




##  MessageHandler


function leftImgHdlr(msgdata, slamw::SLAMWrapperLocal)
  # @show "leftImgHdlr", msgdata[2].header.seq
  leftdata = take!(IOBuffer(msgdata[2].data))
  push!(SyncImages[:left], (msgdata[2].header.seq, ImageMagick.load_(leftdata)) )
  # img = last(SyncImages[:left])
  nothing
end

function rightImgHdlr(msgdata, slamw::SLAMWrapperLocal)
  # @show "rightImgHdlr", msgdata[2].header.seq
  rightdata = take!(IOBuffer(msgdata[2].data))
  push!(SyncImages[:right], (msgdata[2].header.seq, ImageMagick.load_(rightdata)) )
  # img = last(SyncImages[:right])
  nothing
end

function odomHdlr(msgdata, slamw::SLAMWrapperLocal)

  pos = msgdata[2].pose.pose.position
  quat = msgdata[2].pose.pose.orientation

  zPi = [pos.x;pos.y;pos.z]
  zQi = TU.Quaternion(quat.w,[quat.x;quat.y;quat.z])

  zTi = SE3(zPi, zQi)
  push!(SyncImages[:zedodo], (msgdata[2].header.seq, zTi) )

  drawLatestImagePair(SyncImages)
  updateSLAM!(slamw, SyncImages)

  nothing
end





## setup subscriptions to bagfile

slam = SLAMWrapperLocal()
getSolverParams(slam.dfg).drawtree = true


addVariable!(slam.dfg, :x0, Pose2)
addFactor!(slam.dfg, [:x0], PriorPose2(MvNormal(zeros(3),diagm([0.1,0.1,0.01].^2))))

BagSubscriber = RosbagSubscriber(bagfile)

BagSubscriber(leftimgtopic, leftImgHdlr, slam)
BagSubscriber(rightimgtopic, rightImgHdlr, slam)
BagSubscriber(zedodomtopic, odomHdlr, slam)
