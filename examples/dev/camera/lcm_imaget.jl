# using Images, ImageView
using LCMCore
using FixedPointNumbers
using ColorTypes
using ImageView


mutable struct image_metadata_t <: LCMType
    key::String
    n::Int32
    value::Vector{UInt8}
end

@lcmtypesetup(image_metadata_t,
    value => (n,)
)

mutable struct image_t <: LCMType
    utime::Int64
    width::Int32
    height::Int32
    row_stride::Int32
    pixelformat::Int32
    size::Int32
    data::Vector{UInt8}
    nmetadata::Int32
    metadata::Vector{image_metadata_t}
end

@lcmtypesetup(image_t,
    data => (size,),
    metadata => (nmetadata,)
)

# these values are to be kept in sync with the CamPixelFormat enumeration
# in the Camunits library.  See camunits/pixels.h in Camunits
const PIXEL_FORMAT_UYVY             = Int32(1498831189)
const PIXEL_FORMAT_YUYV             = Int32(1448695129)
const PIXEL_FORMAT_IYU1             = Int32(827677001)
const PIXEL_FORMAT_IYU2             = Int32(844454217)
const PIXEL_FORMAT_YUV420           = Int32(842093913)
const PIXEL_FORMAT_YUV411P          = Int32(1345401140)
const PIXEL_FORMAT_I420             = Int32(808596553)
const PIXEL_FORMAT_NV12             = Int32(842094158)
const PIXEL_FORMAT_GRAY             = Int32(1497715271)
const PIXEL_FORMAT_RGB              = Int32(859981650)
const PIXEL_FORMAT_BGR              = Int32(861030210)
const PIXEL_FORMAT_RGBA             = Int32(876758866)
const PIXEL_FORMAT_BGRA             = Int32(877807426)
const PIXEL_FORMAT_BAYER_BGGR       = Int32(825770306)
const PIXEL_FORMAT_BAYER_GBRG       = Int32(844650584)
const PIXEL_FORMAT_BAYER_GRBG       = Int32(861427800)
const PIXEL_FORMAT_BAYER_RGGB       = Int32(878205016)
const PIXEL_FORMAT_BE_BAYER16_BGGR  = Int32(826360386)
const PIXEL_FORMAT_BE_BAYER16_GBRG  = Int32(843137602)
const PIXEL_FORMAT_BE_BAYER16_GRBG  = Int32(859914818)
const PIXEL_FORMAT_BE_BAYER16_RGGB  = Int32(876692034)
const PIXEL_FORMAT_LE_BAYER16_BGGR  = Int32(826360396)
const PIXEL_FORMAT_LE_BAYER16_GBRG  = Int32(843137612)
const PIXEL_FORMAT_LE_BAYER16_GRBG  = Int32(859914828)
const PIXEL_FORMAT_LE_BAYER16_RGGB  = Int32(876692044)
const PIXEL_FORMAT_MJPEG            = Int32(1196444237)
const PIXEL_FORMAT_BE_GRAY16        = Int32(357)
const PIXEL_FORMAT_LE_GRAY16        = Int32(909199180)
const PIXEL_FORMAT_BE_RGB16         = Int32(358)
const PIXEL_FORMAT_LE_RGB16         = Int32(1279412050)
const PIXEL_FORMAT_BE_SIGNED_GRAY16 = Int32(359)
const PIXEL_FORMAT_BE_SIGNED_RGB16  = Int32(360)
const PIXEL_FORMAT_FLOAT_GRAY32     = Int32(842221382)
const PIXEL_FORMAT_INVALID          = Int32(-2)
const PIXEL_FORMAT_ANY              = Int32(-1)


#
# def decode(self, data):
#     msg = image_t.decode(data)
#     if msg.pixelformat == image_t.PIXEL_FORMAT_GRAY:
#         return im_resize(np.asarray(bytearray(msg.data),
#                                     dtype=np.uint8).reshape(msg.height,
#                                                             msg.width),
#                          scale=self.scale)
#     elif msg.pixelformat == image_t.PIXEL_FORMAT_MJPEG:
#         im = cv2.imdecode(np.asarray(bytearray(msg.data),
#                                      dtype=np.uint8),
#                           -1)
#         return im_resize(im, scale=self.scale)
#     else:
#     raise RuntimeError('Unknown pixelformat for ImageDecoder')

# global imgt = image_t()

global canvas = Dict{String, Any}()

function updateCamCallback!(channel::String, msg::image_t)
    global canvas
    imc = Array{RGB{N0f8}}(undef, msg.height, msg.width)
    if (msg.pixelformat == PIXEL_FORMAT_RGB)
        im1 = reshape(msg.data, (3, msg.width, msg.height))
        im2 = reinterpret(N0f8, im1)
        for i in 1:msg.height, j in 1:msg.width
          imc[i, j] = RGB(im2[:, j,i]...)
        end
    end
    if length(canvas) == 0
      canvas = imshow(imc)
    else
      imshow(canvas["gui"]["canvas"], imc)
    end
end

# This colour mapping may be wrong for the PS3eye.
# ycrcb = Video4Linux.YUYV(640,480)
# vidchan = Channel((c::Channel) -> videoproducer(c, ycrcb, devicename = "/dev/video0", iomethod = Video4Linux.IO_METHOD_MMAP ))
#
# ##
# #capture one frame to create im1 and canvas needed to diplay
# A =  take!(vidchan)
# im1 = RGB.(YCbCr.(view(A,:,:,1), view(A,:,:,2), view(A,:,:,3)))

global const lcm_handle = LCM("udpm://239.255.76.67:7667?ttl=1")

subscribe(lcm_handle, "IMAGE_STREAM", updateCamCallback!, image_t)

# canvas = imshow(im1)

while handle(lcm_handle)
end
