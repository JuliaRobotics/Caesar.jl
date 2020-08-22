# test AprilTag detections


using Images, ImageDraw, ImageMagick
using AprilTags

# ENV["PYTHON"] = "python"
# using PyCall
#
# @pyimport numpy as np
# @pyimport cv2

# deprecated, use AprilTags.drawTags instead
function showImage(image, tags, K)
    # Convert image to RGB
    imageCol = RGB.(image)
    #traw color box on tag corners
    foreach(tag->drawTagBox!(imageCol, tag, width = 2, drawReticle = false), tags)
    foreach(tag->drawTagAxes!(imageCol,tag, K), tags)
    imageCol
end

global detector = AprilTagDetector()

# include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","cameraUtils.jl"))

cx = 327.986
cy = 198.066
fx = 349.982
fy = 349.982
K = [-fx 0  cx;
0 fy cy]


img = load(dirname(@__FILE__)*"/example_tag_image.jpeg");
tags = detector(img)
showImage(img, tags, K)

tags

cTt = homographytopose(tags[1].H, fx,fy, cx, cy, taglength=0.172)

# getAprilTagImage(1, AprilTags.tag36h11)



using CoordinateTransformations







#
