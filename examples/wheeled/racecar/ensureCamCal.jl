# trying to see range bearing AprilTag detections

using Images, ImageDraw, ImageView, FileIO
using YAML, JLD, HDF5
using Colors

using AprilTags



include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","racecarUtils.jl"))


cfg = loadConfig()
# lazy bad ... sorry
cw, ch = 330.4173, 196.32587
focal = 340.97913
tagsize = 0.172


datafolder = ENV["HOME"]*"/data/racecar/smallloop/"
@load datafolder*"tag_det_per_pose.jld" tag_bag
# tag_bag[0]

camcount = readdlm(datafolder*"images/cam-count.csv",',')
camlookup = Dict{Int, String}()
for i in 1:size(camcount,1)
  camlookup[camcount[i,1]] = strip(camcount[i,2])
end



IMGS = []


# pose :x0

img = load(datafolder*"images/left0005.jpg")

push!(IMGS, deepcopy(img))

drawTagLine!(IMGS[1], tag_bag[0][0])
drawTagLine!(IMGS[1], tag_bag[0][1])
drawTagLine!(IMGS[1], tag_bag[0][16])
drawTagLine!(IMGS[1], tag_bag[0][584])



IMGS[1]



# pose :x1
psid = 1

pose_tags = tag_bag[psid]
img = load(datafolder*"images/$(camlookup[psid])")
push!(IMGS, deepcopy(img))

drawTagLine!(IMGS[2], pose_tags[0])
drawTagLine!(IMGS[2], pose_tags[1])
drawTagLine!(IMGS[2], pose_tags[16])
drawTagLine!(IMGS[2], pose_tags[584])


IMGS[2]







# pose :x10
psid = 10

pose_tags = tag_bag[psid]
img = load(datafolder*"images/$(camlookup[psid])")
push!(IMGS, deepcopy(img))

drawTagLine!(IMGS[end], pose_tags[0])
drawTagLine!(IMGS[end], pose_tags[1])
drawTagLine!(IMGS[end], pose_tags[16])


IMGS[end]









# pose :x25
psid = 25

pose_tags = tag_bag[psid]
img = load(datafolder*"images/$(camlookup[psid])")
push!(IMGS, deepcopy(img))



drawTagLine!(IMGS[end], pose_tags[8])
drawTagLine!(IMGS[end], pose_tags[9])
drawTagLine!(IMGS[end], pose_tags[10])
drawTagLine!(IMGS[end], pose_tags[16])


IMGS[end]









# pose :x25
psid = 26

pose_tags = tag_bag[psid]
img = load(datafolder*"images/$(camlookup[psid])")
push!(IMGS, deepcopy(img))


drawTagLine!(IMGS[end], pose_tags[9])
drawTagLine!(IMGS[end], pose_tags[10])
drawTagLine!(IMGS[end], pose_tags[16])


IMGS[end]














using TransformUtils

# redetect tags

detector = AprilTagDetector()

tags = detector(img)

pose = homography_to_pose(tags[5].H, -focal, focal, cw, ch)


cV = pose[1:3,4]

bV = cfg[:bRc]*cV



bearing = atan2(-bV[1], bV[3])
# tag_det[id][:range] = [norm(tag_det[id][:pos][[1;3]]);]


foreach(tag->drawTagBox!(img,tag, width = 5, drawReticle = false), tags)


img

#
