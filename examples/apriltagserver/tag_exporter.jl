using Images, ImageView
using Video4Linux
using AprilTags
using FixedPointNumbers
using LCMCore
using CaesarLCMTypes
using ArgParse

function commands()
  s = ArgParseSettings()
  @add_arg_table s begin
      "--device", "-d"
          help = "Device location for camera"
          arg_type = String
          default = "/dev/video1"
      "--frames", "-f"
          help = "number of frames to transmit"
          arg_type = Int
          default = 1000
  end
  parse_args(ARGS, s)
end

function showImage(image, tags)
    # Convert image to RGB
    imageCol = RGB.(image)
    #traw color box on tag corners
    foreach(tag->drawTagBox!(imageCol, tag), tags)

    imageCol
end

function apriltag_converter(a::AprilTag, utime::Int64)
    msg=apriltag_t();
    msg.utime = utime
    msg.id=a.id;
    msg.familyName=a.family;
    msg.hammingDistance=a.hamming;
    P = reshape([a.p...;],2,4)'
    msg.p=P
    msg.cxy= a.c;
    msg.homography= a.H;
    msg.pose = homography_to_pose(a.H, -80.,80.,320.,240.)
    return msg
end

@show parsed_args = commands()

# This colour mapping may be wrong for the PS3eye.
ycrcb = Video4Linux.YUYV(640,480)
vidchan = Channel((c::Channel) -> videoproducer(c, ycrcb, devicename = parsed_args["device"],
                                         iomethod = Video4Linux.IO_METHOD_MMAP, N=parsed_args["frames"] ))

##
#capture one frame to create im1 and canvas needed to diplay

A =  take!(vidchan)
im1 = RGB.(YCbCr.(view(A,:,:,1), view(A,:,:,2), view(A,:,:,3)))
# canvas = imshow(im1)

# Create default detector
detector = AprilTagDetector()


# tags = detector(im1)
lcm=LCM()

while isopen(vidchan)
    A =  take!(vidchan)
    utime = Int64(time()*1000_000)
    im1 = RGB.(YCbCr.(A[:,:,1], A[:,:,2], A[:,:,3]))
    image = Gray{N0f8}.(im1)

    # image = reinterpret.(RGB{N0f8},im1) #ImageMagick function
    tags = detector(image)
    msgs=apriltag_converter.(tags, utime)
    @show msgs
    data=encode.(msgs)

    for da in data
        publish(lcm,"TEST_CHANNEL",da)
    end

    #im2=showImage(image, tags)
    #imshow(canvas["gui"]["canvas"], im2)
end

close(vidchan)
