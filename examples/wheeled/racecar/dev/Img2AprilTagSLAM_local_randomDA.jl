    # Local compute version

# add more julia processes
nprocs() < 5 ? addprocs(5-nprocs()) : nothing

using Caesar, RoME, Distributions
using YAML, JLD, HDF5
using RoMEPlotting, Gadfly
using AprilTags
using Images, ImageView, ImageDraw
using CoordinateTransformations, Rotations, StaticArrays
using MeshCat
using GeometryTypes

const KDE = KernelDensityEstimate
const IIF = IncrementalInference
const TU = TransformUtils

# temporarily using Python / OpenCV for solvePnP
using PyCall

@pyimport numpy as np
@pyimport cv2


include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","racecarUtils.jl"))
include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","cameraUtils.jl"))
include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","visualizationUtils.jl"))


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
imgfolder = "images"


# Figure export folder
currdirtime = now()
imgdir = joinpath(ENV["HOME"], "Pictures", "racecarimgs", "$(currdirtime)")
mkdir(imgdir)
mkdir(imgdir*"/tags")


# process images
camlookup = prepCamLookup(175:5:370)
IMGS, TAGS = detectTagsViaCamLookup(camlookup, datafolder*imgfolder, imgdir)
# IMGS[1]
# TAGS[1]

tag_bag = prepTagBag(TAGS)
# tag_bag[0]


# save the tag bag file for future use
@save imgdir*"/tag_det_per_pose.jld" tag_bag



# Factor graph construction
N = 100
fg = initfg()

psid = 0
pssym = Symbol("x$psid")
# first pose with zero prior
addVariable!(fg, pssym, DynPose2(ut=0))
# addFactor!(fg, [pssym], PriorPose2(MvNormal(zeros(3),diagm([0.01;0.01;0.001].^2))))
addFactor!(fg, [pssym], DynPose2VelocityPrior(MvNormal(zeros(3),diagm([0.01;0.01;0.001].^2)),
                                              MvNormal(zeros(2),diagm([0.3;0.01].^2))))

addApriltags!(fg, pssym, tag_bag[psid], lmtype=Pose2, fcttype=DynPose2Pose2)

# writeGraphPdf(fg)

# quick solve as sanity check
tree = wipeBuildNewTree!(fg, drawpdf=true)
inferOverTreeR!(fg,tree, N=N)

# plotKDE(fg, :l1, dims=[3])

# ls(fg)
#
# val = getVal(fg, :l11)

# drawPosesLandms(fg, spscale=0.25)

# @async run(`evince bt.pdf`)

prev_psid = 0
# add other positions
maxlen = (length(tag_bag)-1)
# psid = 5
for psid in 1:1:maxlen #[5;9;13;17;21;25;29;34;39] #17:4:21 #maxlen
  @show psid
  addnextpose!(fg, prev_psid, psid, tag_bag[psid], lmtype=Pose2, odotype=VelPose2VelPose2, fcttype=DynPose2Pose2, DAerrors=0.2)
  # writeGraphPdf(fg)
  if psid % 3 == 0 || psid == maxlen
    tree = wipeBuildNewTree!(fg, drawpdf=true)
    inferOverTree!(fg,tree, N=N)
  end
    ensureAllInitialized!(fg)
    pl = drawPosesLandms(fg, spscale=0.1, drawhist=false)#,   meanmax=:mean,xmin=-3,xmax=6,ymin=-5,ymax=2);
    Gadfly.draw(PNG(joinpath(imgdir,"x$(psid).png"),15cm, 10cm),pl)
    pl = drawPosesLandms(fg, spscale=0.1)#,   meanmax=:mean,xmin=-3,xmax=3,ymin=-2,ymax=2);
    Gadfly.draw(PNG(joinpath(imgdir,"hist_x$(psid).png"),15cm, 10cm),pl)
    pl = plotPose2Vels(fg, Symbol("x$(psid)"), coord=Coord.Cartesian(xmin=-1.0, xmax=1.0))
    Gadfly.draw(PNG(joinpath(imgdir,"vels_x$(psid).png"),15cm, 10cm),pl)
  prev_psid = psid
end


IIF.savejld(fg, file=imgdir*"/racecar_fg_$(currdirtime).jld")




#
#
# ls(fg, :l7)
#
# stuff = IIF.localProduct(fg, :l7)
# plotKDE(stuff[2], levels=3, dims=[3])
#
# tag_bag[28][7][:tRYc]
# tag_bag[31][7][:tRYc]
#
#
# plotKDE(fg, :l1, dims=[3])
#
# getfnctype(getVert(fg, :x28l7f1, nt=:fnc)).z.μ
# getfnctype(getVert(fg, :x31l7f1, nt=:fnc)).z.μ
#
# ls(fg, :l17)
#
# getfnctype(getVert(fg, :x28l17f1, nt=:fnc)).z.μ
# getfnctype(getVert(fg, :x30l17f1, nt=:fnc)).z.μ
#
#
# # fg, = IncrementalInference.loadjld(file="30jul18_9AM.jld")
# # addFactor!(fg, [:l5;:l16], Point2Point2Range(Normal(0.5,1.0)))
# # ls(fg,:l5)
#
#
#
# fid = open("results.csv","w")
# for sym in [ls(fg)[1]...;ls(fg)[2]...]
#   p = getBelief(fg, sym)
#   val = string(KDE.getKDEMax(p))
#   println(fid, "$sym, $(val[2:(end-1)])")
# end
# close(fid)




0



#
