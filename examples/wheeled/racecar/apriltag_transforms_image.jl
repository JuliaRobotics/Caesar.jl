# tag transforms from image

using YAML
using AprilTags, Images, ImageMagick
using Rotations, CoordinateTransformations
using StaticArrays
using MeshCat
using GeometryTypes

using PyCall
@pyimport numpy as np
@pyimport cv2


include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","cameraUtils.jl"))
include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","racecarUtils.jl"))

cfg = loadConfig()

# fx = fy = 300.0
# cx,cy = (320,240)
k1,k2 = (0.0,0.0)
cx, cy = cfg[:intrinsics][:cx], cfg[:intrinsics][:cy]
fx = fy = cfg[:intrinsics][:fx]
# k1,k2 = cfg[:intrinsics][:k1], cfg[:intrinsics][:k2]

camK = [fx 0 cx; 0 fy cy; 0 0 1]
tagsize = 0.172

# tag extrinsic rotation
Rx = RotX(-pi/2)
Rz = RotZ(-pi/2)
bTc= LinearMap(Rz) ∘ LinearMap(Rx)

detector = AprilTagDetector()

image = load(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","example_tag_image_4.jpeg"))
tags = detector(image)

freeDetector!(detector)



imgd = deepcopy(image)
foreach(tag->drawTagBox!(imgd,tag, width = 5, drawReticle = false), tags)
imgd





# visualization


vis = Visualizer()
open(vis)

setobject!(vis["home"], Triad(1.0))



function drawTagDetection(vis::Visualizer, tagname, Q, T, bTc, bP2t; posename=:test)
  # draw tag triad
  setobject!(vis[currtag], Triad(0.2))
  settransform!(vis[currtag], bTt)

  # draw ray to tag
  v = vis[posename][:lines][tagname]
  geometry = PointCloud(
  Point.([bTt.translation[1];0],
       [bTt.translation[2];0],
       [0.0;0])
  )
  setobject!(v, LineSegments(geometry, LineBasicMaterial()))
  settransform!(v, Translation(0.0,0,0))

  # draw orientation tail for tag
  v = vis[posename][:orient][tagname]
  geometry = PointCloud(
  Point.([0.1;0],
         [0.0;0],
         [0.0;0])
  )
  setobject!(v, LineSegments(geometry, LineBasicMaterial()))
  settransform!(v, bP2t) #
  nothing
end


tidx = 1
currtag = ""
bTt = LinearMap(Translation(0.0,0,0) ∘ Quat(1.0,0,0,0))
for tidx in 1:length(tags)

currtag = "tag$(tags[tidx].id)"

# get relative tag transform
Q,T, = getAprilTagTransform(tags[tidx],
                          camK,
                          k1,
                          k2,
                          tagsize )
cTt = (T ∘ LinearMap(Q))

# tTl = LinearMap(RotY(pi/2)) ∘ LinearMap(RotX(-pi/2))
bTt = bTc ∘ cTt # ∘ tTl

# get Pose2Pose2 tag orientation transform
bP2t = getTagPP2(bTt)
# dt = convert(RotXYZ, bP2t.linear).theta3

drawTagDetection(vis, currtag, Q, T, bTc, bP2t, posename=:x5)


end










#
