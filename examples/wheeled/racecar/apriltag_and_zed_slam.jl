# Local compute version

using Distributed

"""
Ensure the desired number of julia processes are present.
"""
function check_procs_IIF(n::Int)
  if n > 1
    nprocs() < n ? addprocs(n-nprocs()) : nothing
  end
  procs()
end
check_procs_IIF(4) # TODO use IIF.check_procs(4) once available

using Dates, Statistics
using Caesar
using YAML
using JLD2
using CoordinateTransformations, Rotations, StaticArrays

using AprilTags
using Images, ImageView, ImageDraw

using Fontconfig
using Cairo
using Compose
using RoMEPlotting, Gadfly
# using FileIO
# using GeometryTypes # using MeshCat



include(joinpath(dirname(@__FILE__),"racecarUtils.jl"))
include(joinpath(dirname(@__FILE__),"cameraUtils.jl"))
# include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","visualizationUtils.jl"))


cfg = loadConfig()

cw, ch = cfg[:intrinsics][:cx], cfg[:intrinsics][:cy]
fx = fy = 1.0*cfg[:intrinsics][:fx]
camK = [fx 0 cw; 0 fy ch; 0 0 1]
tagsize = 0.172
# k1,k2 = cfg[:intrinsics][:k1], cfg[:intrinsics][:k2] # makes it worse
k1, k2 = 0.0, 0.0

# tag extrinsic rotation
Rx = RotX(-pi/2)
Rz = RotZ(-pi/2)
bTc= LinearMap(Rz) ∘ LinearMap(Rx)


datadir = joinpath(ENV["HOME"],"data","racecar")


# datafolder = ENV["HOME"]*"/data/racecar/straightrun3/"  # 175:5:370
# datafolder = joinpath(datadir,"labrun2"); camidxs =  0:5:1625
# datafolder = ENV["HOME"]*"/data/racecar/labrun3/"; # camidxs =
datafolder = ENV["HOME"]*"/data/racecar/labrun5/"; camidxs =  0:5:1020
# datafolder = ENV["HOME"]*"/data/racecar/labrun6/"; camidxs =  0:5:1795
# datafolder = ENV["HOME"]*"/data/racecar/labfull/"; camidxs =  0:5:1765
imgfolder = "images"



# Figure export folder
currdirtime = now()
# currdirtime = "2018-10-28T23:17:30.067"
# currdirtime = "2018-11-03T22:48:51.924"
currdirtime = "2018-11-07T01:36:52.274"
resultsparentdir = joinpath(datadir, "results")
resultsdir = joinpath(resultsparentdir, "$(currdirtime)")


mkdir(resultsdir)
mkdir(resultsdir*"/tags")
mkdir(resultsdir*"/images")


fid = open(resultsdir*"/readme.txt", "w")
println(fid, datafolder)
println(fid, camidxs)
close(fid)

fid = open(resultsparentdir*"/racecar.log", "a")
println(fid, "$(currdirtime), $datafolder, $(camidxs)")
close(fid)


# process images
# camlookup = prepCamLookup(175:5:370)
camlookup = prepCamLookup(camidxs)


## TODO: udpate to AprilTags.jl
IMGS, TAGS = detectTagsViaCamLookup(camlookup, joinpath(datafolder,imgfolder), resultsdir)
# IMGS[1]
# TAGS[1]

tag_bag = prepTagBag(TAGS)
# tag_bag[0]


# save the tag bag file for future use
@save resultsdir*"/tag_det_per_pose.jld2" tag_bag
# @load resultsdir*"/tag_det_per_pose.jld2" tag_bag

fid=open(resultsdir*"/tags/pose_tags.csv","w")
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
addNode!(fg, pssym, DynPose2(ut=0))
# addFactor!(fg, [pssym], PriorPose2(MvNormal(zeros(3),diagm([0.01;0.01;0.001].^2))))
addFactor!(fg, [pssym], DynPose2VelocityPrior(MvNormal(zeros(3),Matrix(Diagonal([0.01;0.01;0.001].^2))),
                                              MvNormal(zeros(2),Matrix(Diagonal([0.1;0.05].^2)))))

addApriltags!(fg, pssym, tag_bag[psid], lmtype=Pose2, fcttype=DynPose2Pose2)

writeGraphPdf(fg)

# quick solve as sanity check
tree = batchSolve!(fg, N=N, drawpdf=true, show=true)

# @async run(`evince /tmp/bt.pdf`)


# plotKDE(fg, :l1, dims=[3])
# ls(fg)
# val = getVal(fg, :l11)
# drawPosesLandms(fg, spscale=0.25)
# @async run(`evince bt.pdf`)


# add other positions
global maxlen = (length(tag_bag)-1)
global prev_psid = 0

Gadfly.push_theme(:default)


for psid in 1:1:maxlen
  global prev_psid
  global maxlen
  @show psym = Symbol("x$psid")
  addnextpose!(fg, prev_psid, psid, tag_bag[psid], lmtype=Pose2, odotype=VelPose2VelPose2, fcttype=DynPose2Pose2, autoinit=true)
  # writeGraphPdf(fg)


  if psid % 20 == 0 || psid == maxlen
    IIF.savejld(fg, file=resultsdir*"/racecar_fg_$(psym)_presolve.jld2")
    tree = batchSolve!(fg, drawpdf=true, show=true, N=N)
  end
  IIF.savejld(fg, file=resultsdir*"/racecar_fg_$(psym).jld2")

  ## save factor graph for later testing and evaluation
  ensureAllInitialized!(fg)
  pl = drawPosesLandms(fg, spscale=0.1, drawhist=false, meanmax=:mean) #,xmin=-3,xmax=6,ymin=-5,ymax=2);
  Gadfly.draw(PNG(joinpath(resultsdir, "images", "$(psym).png"),15cm, 10cm),pl)
  pl = drawPosesLandms(fg, spscale=0.1, meanmax=:mean) # ,xmin=-3,xmax=3,ymin=-2,ymax=2);
  Gadfly.draw(PNG(joinpath(resultsdir, "images", "hist_$(psym).png"),15cm, 10cm),pl)
  pl = plotPose2Vels(fg, Symbol("$(psym)"), coord=Coord.Cartesian(xmin=-1.0, xmax=1.0))
  Gadfly.draw(PNG(joinpath(resultsdir, "images", "vels_$(psym).png"),15cm, 10cm),pl)

  # prepare for next iteration
  prev_psid = psid
end


IIF.savejld(fg, file=resultsdir*"/racecar_fg_final.jld2")
# fg, = loadjld(file=resultsdir*"/racecar_fg_x280_presolve.jld2")
# fg, = loadjld(file=resultsdir*"/racecar_fg_x159.jld2")



results2csv(fg; dir=resultsdir, filename="results_x159.csv")


# save factor graph for later testing and evaluation
# fg, = loadjld(file=resultsdir*"/racecar_fg_final.jld2")
@time tree = batchSolve!(fg, N=N, drawpdf=true, show=true, recursive=true)

IIF.savejld(fg, file=resultsdir*"/racecar_fg_final_resolve.jld2")
# fgr, = loadjld(file=resultsdir*"/racecar_fg_final_resolve.jld2")
results2csv(fg; dir=resultsdir, filename="results_resolve.csv")


# pl = plotKDE(fg, :x1, levels=1, dims=[1;2]);



#,xmin=-3,xmax=6,ymin=-5,ymax=2);
Gadfly.push_theme(:default)
pl = drawPosesLandms(fg, spscale=0.1, drawhist=false, meanmax=:max);
# Gadfly.set(:default_theme)
Gadfly.draw(PNG(joinpath(resultsdir,"images","final.png"),15cm, 10cm),pl);
Gadfly.draw(SVG(joinpath(resultsdir,"images","final.svg"),15cm, 10cm),pl);

# pl = drawPosesLandms(fg, spscale=0.1, meanmax=:mean) # ,xmin=-3,xmax=3,ymin=-2,ymax=2);
# Gadfly.draw(PNG(joinpath(resultsdir,"hist_final.png"),15cm, 10cm),pl)

0




# debugging

# using Profile
#
# using Logging
# global_logger(NullLogger())
# @profile println("test")
# Profile.clear()
# @profile tree1 = IIF.wipeBuildNewTree!(fg)
#
# Juno.profiler()


# vars = lsRear(fg, 5)
#
#
# num_edges(fg.g)
#
# subgraphFromVerts(fg, vars)
#
# ensureAllInitialized!(fg)
# isInitialized(fg, :x220)
#
# ls(fg, :l14)


#
