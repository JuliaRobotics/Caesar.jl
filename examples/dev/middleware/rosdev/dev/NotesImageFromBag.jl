# loading images

# python 2.7 at this time
from rosbagReader import *

reader = RosbagParser(".../file.bag", "/zed/zed_node/left/image_rect_color/compressed")

msg = reader.get_next_message()

data = msg[1].data

using ImageMagick

data8 = UInt8.(data)

img = ImageMagick.load_(data8)


using ImageView

imshow(img)




## again

reader = RosbagParser(bagfile, leftimgtopic)


msg = reader.get_next_message()
msg[2].header.seq
msg[3]
leftdata = take!(IOBuffer(msg[2].data))
img = ImageMagick.load_(leftdata)

typeof(img)

msg[end]

imshow(img)
