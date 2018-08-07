    # Local compute version

# add more julia processes
nprocs() < 5 ? addprocs(5-nprocs()) : nothing

using Caesar, RoME, Distributions
using YAML, JLD, HDF5
using RoMEPlotting, Gadfly
using AprilTags
using Images, ImageView, ImageDraw
using CoordinateTransformations, Rotations, StaticArrays

const KDE = KernelDensityEstimate
const IIF = IncrementalInference
const TU = TransformUtils

using PyCall

@pyimport numpy as np
@pyimport cv2


include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","racecarUtils.jl"))

include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","cameraUtils.jl"))


cfg = loadConfig()

cw, ch = cfg[:intrinsics][:cx], cfg[:intrinsics][:cy]
fx = fy = cfg[:intrinsics][:fx]
camK = [fx 0 cw; 0 fy ch; 0 0 1]
tagsize = 0.172
# k1,k2 = cfg[:intrinsics][:k1], cfg[:intrinsics][:k2] # makes it worse
k1, k2 = 0.0, 0.0

# tag extrinsic rotation
Rx = RotX(-pi/2)
Rz = RotZ(-pi/2)
bTc= LinearMap(Rz) ∘ LinearMap(Rx)

datafolder = ENV["HOME"]*"/data/racecar/straightrun3/"
# tag_bag[0]


imgfolder = "images"
# camcount = readdlm(datafolder*"$(imgfolder)/cam-count.csv",',')
camlookup = Dict{Int, String}()
count = -1
for i in 175:5:370
  count += 1
  camlookup[count] = "camera_image$(i).jpeg"
  # camlookup[camcount[i,1]] = strip(camcount[i,2])
end



# Figure export folder
currdirtime = now()
imgdir = joinpath(ENV["HOME"], "Pictures", "racecarimgs", "$(currdirtime)")
mkdir(imgdir)
mkdir(imgdir*"/tags")


# AprilTag detector
detector = AprilTagDetector()

# extract tags from images
IMGS = []
TAGS = []
psid = 1
for psid in 0:(length(camlookup)-1)
  img = load(datafolder*"$(imgfolder)/$(camlookup[psid])")
  tags = detector(img)
  push!(TAGS, deepcopy(tags))
  push!(IMGS, deepcopy(img))
  foreach(tag->drawTagBox!(IMGS[psid+1],tag, width = 5, drawReticle = false), tags)
  save(imgdir*"/tags/img_$(psid).jpg", IMGS[psid+1])
end
# psid = 1
# img = load(datafolder*"$(imgfolder)/$(camlookup[psid])")

# free the detector memory
freeDetector!(detector)


# IMGS[1]
# TAGS[1]
tvec = Translation(0.0,0,0)
q = Quat(1.0,0,0,0)
# package tag detections for all keyframes in a tag_bag
tag_bag = Dict{Int, Any}()
for psid in 0:(length(TAGS)-1)
  tag_det = Dict{Int, Any}()
  for tag in TAGS[psid+1]
    q, tvec = getAprilTagTransform(tag, camK, k1, k2, tagsize)
    cTt = tvec ∘ CoordinateTransformations.LinearMap(q)
    tag_det[tag.id] = buildtagdict(cTt, q, tvec, tagsize, bTc)
  end
  tag_bag[psid] = tag_det
end
tag_bag[0]
TAGS[1]


# save the tag bag file for future use
@save imgdir*"/tag_det_per_pose.jld" tag_bag



# Factor graph construction
N = 75
fg = initfg()

psid = 0
pssym = Symbol("x$psid")
# first pose with zero prior
addNode!(fg, pssym, Pose2)
# addFactor!(fg, [pssym], PriorPose2(MvNormal(zeros(3),diagm([0.01;0.01;0.001].^2))))
addFactor!(fg, [pssym], DynPose2VelocityPrior(MvNormal(zeros(3),diagm([0.01;0.01;0.001].^2)),
                                              MvNormal(zeros(2),diagm([0.01;0.01].^2))))

addApriltags!(fg, pssym, tag_bag[psid], lmtype=Pose2)

# writeGraphPdf(fg)

# quick solve as sanity check
tree = wipeBuildNewTree!(fg, drawpdf=true)
inferOverTreeR!(fg,tree, N=N)

plotKDE(fg, :l1, dims=[3])

# ls(fg)
#
# val = getVal(fg, :l11)

drawPosesLandms(fg, spscale=0.25)

# @async run(`evince bt.pdf`)

prev_psid = 0
# add other positions
maxlen = (length(tag_bag)-1)
for psid in [5;9;13;17] #17:4:21 #maxlen
  addnextpose!(fg, prev_psid, psid, tag_bag[psid], lmtype=Pose2, odotype=DynPose2)
  # writeGraphPdf(fg)
  if psid % 1 == 0 || psid == maxlen
    tree = wipeBuildNewTree!(fg, drawpdf=true)
    inferOverTree!(fg,tree, N=N)

    pl = drawPosesLandms(fg, spscale=0.1, drawhist=false)#,   meanmax=:mean,xmin=-3,xmax=6,ymin=-5,ymax=2);
    Gadfly.draw(PNG(joinpath(imgdir,"x$(psid).png"),30cm, 25cm),pl)
    pl = drawPosesLandms(fg, spscale=0.1)#,   meanmax=:mean,xmin=-3,xmax=3,ymin=-2,ymax=2);
    Gadfly.draw(PNG(joinpath(imgdir,"hist_x$(psid).png"),30cm, 25cm),pl)
  end
  prev_psid = psid
end

IIF.savejld(fg, file=imgdir*"/racecar_fg_$(currdirtime).jld")

ls(fg, :l7)

stuff = IIF.localProduct(fg, :l7)
plotKDE(stuff[2], levels=3, dims=[3])

tag_bag[28][7][:tRYc]
tag_bag[31][7][:tRYc]


plotKDE(fg, :l1, dims=[3])

getfnctype(getVert(fg, :x28l7f1, nt=:fnc)).z.μ
getfnctype(getVert(fg, :x31l7f1, nt=:fnc)).z.μ

ls(fg, :l17)

getfnctype(getVert(fg, :x28l17f1, nt=:fnc)).z.μ
getfnctype(getVert(fg, :x30l17f1, nt=:fnc)).z.μ


# fg, = IncrementalInference.loadjld(file="30jul18_9AM.jld")
# addFactor!(fg, [:l5;:l16], Point2Point2Range(Normal(0.5,1.0)))
# ls(fg,:l5)



fid = open("results.csv","w")
for sym in [ls(fg)[1]...;ls(fg)[2]...]
  p = getVertKDE(fg, sym)
  val = string(KDE.getKDEMax(p))
  println(fid, "$sym, $(val[2:(end-1)])")
end
close(fid)




0



#
