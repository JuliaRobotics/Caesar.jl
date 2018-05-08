using Images, ImageView
using Video4Linux
using AprilTags
using FixedPointNumbers
using LCMCore
using CaesarLCMTypes

function showImage(image, tags)
    # Convert image to RGB
    imageCol = RGB.(image)
    #traw color box on tag corners
    foreach(tag->drawTagBox!(imageCol, tag), tags)

    imageCol
end

function apriltag_converter(a::AprilTag)
    msg=apriltag_t();
    msg.id=a.id;
    msg.familyName=a.family;
    msg.hammingDistance=a.hamming;
    P = reshape([a.p...;],2,4)'
    msg.p=P
    msg.cxy= a.c;
    msg.homography= a.H;
    msg.pose = homography_to_pose(a.H, -100.,100.,320.,240.)
    return msg
end



# This colour mapping may be wrong for the PS3eye.
ycrcb = Video4Linux.YUYV(640,480)
vidchan = Channel((c::Channel) -> videoproducer(c, ycrcb, devicename = "/dev/video2",
                                         iomethod = Video4Linux.IO_METHOD_MMAP, N=100 ))

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
    im1 = RGB.(YCbCr.(A[:,:,1], A[:,:,2], A[:,:,3]))
    image = Gray{N0f8}.(im1)


    # image = reinterpret.(RGB{N0f8},im1) #ImageMagick function
    tags = detector(image)

    msgs=apriltag_converter.(tags)
    @show msgs
    data=encode.(msgs)

    for da in data
        publish(lcm,"TEST_CHANNEL",da)
    end



    #im2=showImage(image, tags)
    #imshow(canvas["gui"]["canvas"], im2)
end

close(vidchan)
