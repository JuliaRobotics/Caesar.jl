# trying to see range bearing AprilTag detections

using Images, ImageDraw, ImageView, FileIO
using YAML, JLD, HDF5
using Colors

using AprilTags



include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","racecarUtils.jl"))


cfg = loadConfig()
# some default, probably not right
# cw, ch = 330.4173, 196.32587
# focal = 340.97913
# tagsize = 0.172

cw, ch = cfg[:intrinsics][:cx], cfg[:intrinsics][:cy]
focal = cfg[:intrinsics][:fx]
tagsize = 0.172



# datafolder = ENV["HOME"]*"/data/racecar/smallloop/"
datafolder = ENV["HOME"]*"/data/racecar/thursday/"
@load datafolder*"tag_det_per_pose.jld" tag_bag
# tag_bag[0]

imgfolder = "newCalibrationTest3"
# camcount = readdlm(datafolder*"$(imgfolder)/cam-count.csv",',')
camlookup = Dict{Int, String}()
for i in 1:length(tag_bag)
  camlookup[i-1] = "camera_image$(5*(i-1)).jpeg"
  # camlookup[camcount[i,1]] = strip(camcount[i,2])
end





IMGS = []
psid = 1
for psid in 0:(length(tag_bag)-1)
  img = load(datafolder*"$(imgfolder)/$(camlookup[psid])")
  push!(IMGS, deepcopy(img))
  for (tid, vals) in tag_bag[psid]
    drawTagLine!(IMGS[psid+1], vals)
  end
end



IMGS[1]
IMGS[20]
IMGS[30]
IMGS[40]
IMGS[50]
IMGS[59]
IMGS[60]
IMGS[61]
IMGS[62]
IMGS[63]
IMGS[64]
IMGS[70]
IMGS[71]
IMGS[72]
IMGS[73]
IMGS[74]
IMGS[75]
IMGS[76]
IMGS[77]
IMGS[78]
IMGS[79]
IMGS[87]
IMGS[88]
IMGS[90]
IMGS[99]
IMGS[100]
IMGS[101]
IMGS[102]
IMGS[109]
IMGS[110]
IMGS[119]
IMGS[120]
IMGS[121]
IMGS[122]
IMGS[123]
IMGS[124]
IMGS[125]
IMGS[126]
IMGS[127]
IMGS[128]
IMGS[129]
IMGS[130]
IMGS[131]
IMGS[132]
IMGS[139]
IMGS[140]
IMGS[149]
IMGS[150]
IMGS[159]
IMGS[160]
IMGS[163]



tag_bag[60][18][:bearing]
tag_bag[61][18][:bearing]
tag_bag[62][18][:bearing]
tag_bag[63][18][:bearing]

tag_bag[77]

tag_bag[86]
tag_bag[87]
tag_bag[88]

println("$(tag_bag[98][0][:bearing]), $(tag_bag[98][0][:range])")
println("$(tag_bag[99][0][:bearing]), $(tag_bag[99][0][:range])")
println("$(tag_bag[100][0][:bearing]), $(tag_bag[100][0][:range])")
println("$(tag_bag[101][0][:bearing]), $(tag_bag[101][0][:range])")
println("$(tag_bag[102][0][:bearing]), $(tag_bag[102][0][:range])")

println("$(tag_bag[98][1][:bearing]), $(tag_bag[98][1][:range])")
println("$(tag_bag[99][1][:bearing]), $(tag_bag[99][1][:range])")
println("$(tag_bag[100][1][:bearing]), $(tag_bag[100][1][:range])")
println("$(tag_bag[101][1][:bearing]), $(tag_bag[101][1][:range])")
println("$(tag_bag[102][1][:bearing]), $(tag_bag[102][1][:range])")



tag_bag[109]
tag_bag[110]

tag_bag[123]

IMGS[125]
tag_bag[124]

tag_bag[125]

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
