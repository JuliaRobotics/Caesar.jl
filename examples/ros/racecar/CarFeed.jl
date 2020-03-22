## BEFORE RUNNING THIS SCRIPT, MAKE SURE ros is in the environment

# source /opt/ros/melodic/setup.bash


## Project path variables

projdir = @__DIR__


## Prepare python version

using Pkg

# ENV["PYTHON"] = "/usr/bin/python3.6"
ENV["PYTHON"] = "/usr/bin/python"
Pkg.build("PyCall")
using PyCall

## Load rosbag reader

include( joinpath(projdir,"..","Utils","RosbagSubscriber.jl") )


## prepare ROS integration

using RobotOS

# standard types
@rosimport sensor_msgs.msg: CompressedImage

rostypegen()

### Rest of Julia types

using LinearAlgebra
using ImageMagick
using ImageView
using ImageFeatures, ImageDraw, Images, CoordinateTransformations
using DataStructures
using ColorTypes, FixedPointNumbers
using AprilTags

using Gtk.ShortNames

using Caesar, RoME, IncrementalInference


## Constant parameters

# using JSON2
const ImgType = Array{ColorTypes.RGB{FixedPointNumbers.Normed{UInt8,8}},2}

# Consume rosbag with car data
bagfile = joinpath(ENV["HOME"],"data/racecar/mrg/lab_run.bag")
leftimgtopic = "/zed/zed_node/left/image_rect_color/compressed"
# leftimgtopic = "/zed/zed_node/left_raw/image_raw_color/compressed"
rightimgtopic = "/zed/zed_node/right/image_rect_color/compressed"

# from bagfile
fx = 340.59393310546875
fy = 340.59393310546875
cx = 330.41748046875
cy = 196.3251953125

K = [-fx 0  cx;
      0 fy cy]


## Working memory

gui = imshow_gui((600, 100), (1, 2))  # 2 columns, 1 row of images (each initially 300×300)
canvases = gui["canvas"]

SyncImages = Dict{Symbol, CircularBuffer{Tuple{Int, ImgType}}}()
SyncImages[:left] = CircularBuffer{Tuple{Int, ImgType}}(30)
SyncImages[:right] = CircularBuffer{Tuple{Int, ImgType}}(30)

detector = AprilTagDetector()


## User functions

function showImage(image, tags, K)
    # Convert image to RGB
    imageCol = RGB.(image)
    #traw color box on tag corners
    foreach(tag->drawTagBox!(imageCol, tag, width = 2, drawReticle = false), tags)
    foreach(tag->drawTagAxes!(imageCol,tag, K), tags)
    imageCol
end

function getSyncLatestPair(syncImgs)
  # get all sequence numbers
  lenL = length(syncImgs[:left])
  lenR = length(syncImgs[:right])
  seqL = (x->syncImgs[:left][x][1]).(1:lenL)
  seqR = (x->syncImgs[:right][x][1]).(1:lenR)
  # find greatest common number
  seqLR = intersect(seqL, seqR)
  if 0 == length(seqLR)
    return nothing
  end
  seq = maximum(seqLR)
  idxL = findfirst(x->x==seq, seqL)
  idxR = findfirst(x->x==seq, seqR)
  (idxL, idxR)
end

function drawLatestImagePair(syncImgs)
  # get synced images
  idxL, idxR = getSyncLatestPair(syncImgs)

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


mutable struct SLAMWrapperLocal{G <: AbstractDFG} <: AbstractSLAM
  dfg::G
  poseCount::Int
  frameCounter::Int
  poseStride::Int # pose every frameStride (naive trigger)
  solveStride::Int # solve every solveStride pose
end

SLAMWrapperLocal(;dfg::G=initfg(),
                  poseCount::Int=0,
                  frameCounter::Int=0,
                  poseStride::Int=10,
                  solveStride::Int=10 ) where {G <: AbstractDFG}= SLAMWrapperLocal{G}(dfg,
                      poseCount, frameCounter, poseStride, solveStride)
#


function updateSLAM!(slamw::SLAMWrapperLocal, syncImgs)
  idxL, idxR = getSyncLatestPair(syncImgs)
  # already have a counter
  slamw.frameCounter = syncImgs[:left][idxL][1]
  if slamw.frameCounter % slamw.poseStride != 0
    return nothing
  end

  # continue to add new pose
  prevpose = Symbol("x$(slamw.poseCount)")
  slamw.poseCount += 1
  newpose = Symbol("x$(slamw.poseCount)")
  addVariable!(slamw.dfg, newpose, Pose2)
  pp = Pose2Pose2(MvNormal([0.1;0.0;0.0], diagm([0.4; 0.2; 0.3].^2)))
  addFactor!(slamw.dfg, [prevpose;newpose], pp)

  nothing
end



function manageSolves!(slamw::SLAMWrapperLocal)

  @info "logpath=$(getLogPath(dfg))"
  getSolverParams(dfg).drawtree = true
  getSolverParams(dfg).qfl = dashboard[:SOLVESTRIDE]
  getSolverParams(dfg).isfixedlag = true
  getSolverParams(dfg).limitfixeddown = limitfixeddown

  # allow async process
  # getSolverParams(dfg).async = true

  # prep with empty tree
  tree = emptyBayesTree()





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
  drawLatestImagePair(SyncImages)
  updateSLAM!(slamw, SyncImages)
  nothing
end



## setup subscriptions to bagfile

slam = SLAMWrapperLocal()

addVariable!(slam.dfg, :x0, Pose2)
addFactor!(slam.dfg, [:x0], PriorPose2(MvNormal(zeros(3),diagm([0.1,0.1,0.01].^2))))

BagSubscriber = RosbagSubscriber(bagfile)

BagSubscriber(leftimgtopic, leftImgHdlr, slam)
BagSubscriber(rightimgtopic, rightImgHdlr, slam)


##


loop!(BagSubscriber)
loop!(BagSubscriber)
sleep(0.01)  # allow gui sime time to setup
for i in 1:1000 loop!(BagSubscriber) end



#
