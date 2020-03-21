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

using ImageMagick
using ImageView
using DataStructures
using ColorTypes, FixedPointNumbers
using AprilTags

using Gtk.ShortNames


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


function drawLatestImagePair()
  # get all sequence numbers
  lenL = length(SyncImages[:left])
  lenR = length(SyncImages[:right])
  seqL = (x->SyncImages[:left][x][1]).(1:lenL)
  seqR = (x->SyncImages[:right][x][1]).(1:lenR)
  # find greatest common number
  seqLR = intersect(seqL, seqR)
  if 0 == length(seqLR)
    return nothing
  end
  seq = maximum(seqLR)
  idxL = findfirst(x->x==seq, seqL)
  idxR = findfirst(x->x==seq, seqR)

  # Do apriltag detection
  tagsL = detector(SyncImages[:left][idxL][2])
  tagsR = detector(SyncImages[:right][idxR][2])

  # @show poses = (T->homographytopose(T.H, fx, fy, cx, cy, taglength = 160.)).(tagsL)

  imgLt = showImage(SyncImages[:left][idxL][2], tagsL, K)
  imgRt = showImage(SyncImages[:right][idxR][2], tagsR, K)

  # draw both
  imshow(canvases[1,1], imgLt)
  imshow(canvases[1,2], imgRt)
  # imshow(canvases[1,1], SyncImages[:left][idxL][2])
  # imshow(canvases[1,2], SyncImages[:right][idxR][2])

  Gtk.showall(gui["window"])
  nothing
end

##  MessageHandler

function leftImgHdlr(msgdata)
  # @show "leftImgHdlr", msgdata[2].header.seq
  leftdata = take!(IOBuffer(msgdata[2].data))
  push!(SyncImages[:left], (msgdata[2].header.seq, ImageMagick.load_(leftdata)) )
  # img = last(SyncImages[:left])
  nothing
end

function rightImgHdlr(msgdata)
  # @show "rightImgHdlr", msgdata[2].header.seq
  rightdata = take!(IOBuffer(msgdata[2].data))
  push!(SyncImages[:right], (msgdata[2].header.seq, ImageMagick.load_(rightdata)) )
  # img = last(SyncImages[:right])
  drawLatestImagePair()
  nothing
end


## setup subscriptions to bagfile

BagSubscriber = RosbagSubscriber(bagfile)

BagSubscriber(leftimgtopic, leftImgHdlr)
BagSubscriber(rightimgtopic, rightImgHdlr)


##


loop!(BagSubscriber)
loop!(BagSubscriber)
sleep(0.01)  # allow gui sime time to setup
for i in 1:1000 loop!(BagSubscriber) end



#
