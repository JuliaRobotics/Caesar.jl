    # Local compute version

# add more julia processes
nprocs() < 4 ? addprocs(4-nprocs()) : nothing

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


# datafolder = ENV["HOME"]*"/data/racecar/straightrun3/"  # 175:5:370
# datafolder = ENV["HOME"]*"/data/racecar/labrun2/"
# datafolder = ENV["HOME"]*"/data/racecar/labrun3/"
datafolder = ENV["HOME"]*"/data/racecar/labrun5/" # 0:5:1020
# datafolder = ENV["HOME"]*"/data/racecar/labrun6/" # 0:5:1795
# datafolder = ENV["HOME"]*"/data/racecar/labfull/"  # 0:5:1765
imgfolder = "images"


# Figure export folder
currdirtime = now()
# currdirtime = "2018-08-14T00:52:01.534"
# currdirtime = "2018-09-10T09:22:00.922"
resultsdir = joinpath(ENV["HOME"], "Pictures", "racecarimgs")
imgdir = joinpath(resultsdir, "$(currdirtime)")

mkdir(imgdir)
mkdir(imgdir*"/tags")
mkdir(imgdir*"/images")

camidxs = 0:5:1020

fid = open(imgdir*"/readme.txt", "w")
println(fid, datafolder)
println(fid, camidxs)
close(fid)

fid = open(resultsdir*"/racecar.log", "a")
println(fid, "$(currdirtime), $datafolder")
close(fid)


# process images
# camlookup = prepCamLookup(175:5:370)
camlookup = prepCamLookup(camidxs)
IMGS, TAGS = detectTagsViaCamLookup(camlookup, datafolder*imgfolder, imgdir)
# IMGS[1]
# TAGS[1]

tag_bag = prepTagBag(TAGS)
# tag_bag[0]


# save the tag bag file for future use
@save imgdir*"/tag_det_per_pose.jld" tag_bag
# @load imgdir*"/tag_det_per_pose.jld" tag_bag

fid=open(imgdir*"/tags/pose_tags.csv","w")
for pose in sort(collect(keys(tag_bag)))
  println(fid, "$pose, $(collect(keys(tag_bag[pose])))")
end
close(fid)


# Factor graph construction
N = 100
fg = initfg()

psid = 0
pssym = Symbol("x$psid")
# first pose with zero prior
addVariable!(fg, pssym, DynPose2(ut=0))
# addFactor!(fg, [pssym], PriorPose2(MvNormal(zeros(3),diagm([0.01;0.01;0.001].^2))))
addFactor!(fg, [pssym], DynPose2VelocityPrior(MvNormal(zeros(3),diagm([0.01;0.01;0.001].^2)),
                                              MvNormal(zeros(2),diagm([0.1;0.05].^2))))

addApriltags!(fg, pssym, tag_bag[psid], lmtype=Pose2, fcttype=DynPose2Pose2)

# writeGraphPdf(fg)

# quick solve as sanity check
tree = wipeBuildNewTree!(fg, drawpdf=true)
inferOverTreeR!(fg,tree, N=N)


# plotKDE(fg, :l1, dims=[3])
# ls(fg)
# val = getVal(fg, :l11)
# drawPosesLandms(fg, spscale=0.25)
# @async run(`evince bt.pdf`)


prev_psid = 0
# add other positions
maxlen = (length(tag_bag)-1)
# psid = 5
for psid in 1:1:maxlen #[5;9;13;17;21;25;29;34;39] #17:4:21 #maxlen
  @show psym = Symbol("x$psid")
  addnextpose!(fg, prev_psid, psid, tag_bag[psid], lmtype=Pose2, odotype=VelPose2VelPose2, fcttype=DynPose2Pose2, autoinit=true)
  # writeGraphPdf(fg)


  if psid % 50 == 0 || psid == maxlen
    IIF.savejld(fg, file=imgdir*"/racecar_fg_$(psym)_presolve.jld")
    # tree = wipeBuildNewTree!(fg, drawpdf=true)
    # inferOverTree!(fg,tree, N=N)
  end
  IIF.savejld(fg, file=imgdir*"/racecar_fg_$(psym).jld")

  ## save factor graph for later testing and evaluation
  # ensureAllInitialized!(fg)
  # pl = drawPosesLandms(fg, spscale=0.1, drawhist=false, meanmax=:mean) #,xmin=-3,xmax=6,ymin=-5,ymax=2);
  # Gadfly.draw(PNG(joinpath(imgdir,"$(psym).png"),15cm, 10cm),pl)
  # pl = drawPosesLandms(fg, spscale=0.1, meanmax=:mean) # ,xmin=-3,xmax=3,ymin=-2,ymax=2);
  # Gadfly.draw(PNG(joinpath(imgdir,"hist_$(psym).png"),15cm, 10cm),pl)
  # pl = plotPose2Vels(fg, Symbol("$(psym)"), coord=Coord.Cartesian(xmin=-1.0, xmax=1.0))
  # Gadfly.draw(PNG(joinpath(imgdir,"vels_$(psym).png"),15cm, 10cm),pl)

  # prepare for next iteration
  prev_psid = psid
end


IIF.savejld(fg, file=imgdir*"/racecar_fg_final.jld")
# fg, = loadjld(file=imgdir*"/racecar_fg_final.jld")
results2csv(fg; dir=imgdir, filename="results.csv")


# save factor graph for later testing and evaluation
solveTree!(fg)

IIF.savejld(fg, file=imgdir*"/racecar_fg_final_resolve.jld")
# fgr, = loadjld(file=imgdir*"/racecar_fg_final_resolve.jld")
results2csv(fg; dir=imgdir, filename="results_resolve.csv")



#,xmin=-3,xmax=6,ymin=-5,ymax=2);
Gadfly.push_theme(:default)
pl = drawPosesLandms(fg, spscale=0.1, drawhist=false, meanmax=:max)
# Gadfly.set(:default_theme)
Gadfly.draw(SVG(joinpath(imgdir,"images","final.svg"),15cm, 10cm),pl)
# pl = drawPosesLandms(fg, spscale=0.1, meanmax=:mean) # ,xmin=-3,xmax=3,ymin=-2,ymax=2);
# Gadfly.draw(PNG(joinpath(imgdir,"hist_final.png"),15cm, 10cm),pl)




# artificial loops  THIS IS FOR labrun 5
ppr = Point2Point2Range(Distributions.Normal(1.0, 0.5))
pprm = Point2Point2Range(Distributions.Normal(2.0, 1.0))

addFactor!(fg, [:l12; :l10], ppr)
addFactor!(fg, [:l12; :l16], ppr)
addFactor!(fg, [:l7; :l2], ppr)

addFactor!(fg, [:l1; :l16], pprm)
addFactor!(fg, [:l1; :l10], pprm)
addFactor!(fg, [:l1; :l2], pprm)

addFactor!(fg, [:l0; :l16], pprm)

addFactor!(fg, [:l6; :l9], pprm)

addFactor!(fg, [:l6; :l3], ppr)
addFactor!(fg, [:l6; :l19], ppr)
addFactor!(fg, [:l12; :l3], ppr)
addFactor!(fg, [:l12; :l19], ppr)

ensureAllInitialized!(fg)


IIF.savejld(fg, file=imgdir*"/racecar_fg_presolve.jld")
results2csv(fg; dir=imgdir, filename="results_presolve_nloops.csv")

solveTree!(fg)

IIF.savejld(fg, file=imgdir*"/racecar_fg_solved1_nloops.jld")
results2csv(fg; dir=imgdir, filename="results_solved_nloops.csv")
# Gadfly.push_theme(:default)
pl = drawPosesLandms(fg, spscale=0.1, drawhist=false, meanmax=:max)
Gadfly.draw(SVG(joinpath(imgdir,"images","solve_nloops1.svg"),15cm, 10cm),pl)



drawPoses(fg, spscale=1.0)
drawPosesLandms(fg, spscale=0.5)


getAllLandmGifs(fg, IMGS, show=false, dir=imgdir*"/images/")


ls(fg)

poserange = 1:39




plotPoseVelAsMax(fg, 1:39)



plotPose(fg, [:x1;:x5;:x10;:x15;:x20;:x25;:x30;:x35], levels=1)


plotPose(fg, [:x15;:x20], levels=1)





sym = :x2
ls(fg, sym)
pl = plotLocalProduct(fg, sym, dims=[1])
Gadfly.draw(SVG(joinpath(imgdir,"images","localp_$sym.svg"),15cm, 3*10cm),pl)


stuff = IIF.localProduct(fg, :l0)


P = *(stuff[3][1])

plotKDE([P;stuff[3][1]], c=["red";["cyan" for j in 1:length(stuff[3][1])]])

getVal(fg, :l1)



##




genGifLandm(fg, :l1, IMGS)



fsym = :x0l0f1
getData(getVert(fg, fsym,nt=:fnc)).fnc.usrfnc!.Zpose.z.μ

XX, LL = (KDE.getKDEMax.(IIF.getBelief.(fg, IIF.lsf(fg, fsym)))...)
@show xyt = se2vee(SE2(XX[1:3]) \ SE2([LL[1:2];0.0]))
bear= TU.wrapRad(atan2(-xyt[2],xyt[1]) - XX[3])




im = imageFactor(fg, :x0l1f1, IMGS[1], cfg)
BRIMGS = Dict{Symbol,Dict{Symbol, Any}}()
BRIMGS[:x0] = Dict{Symbol,Any}()
BRIMGS[:x0][:l1] = im

BIM = drawAllBearingImgs(fg, IMGS, cfg)

BIM[:x0][:l1]

sym = :l1


imageFactor(fg, :x3l0f1, IMGS[4], cfg)
imageFactor(fg, :x3l1f1, IMGS[4], cfg)
imageFactor(fg, :x3l2f1, IMGS[4], cfg)



0






#
