# Local compute version

@show ARGS

#include("parsecommands.jl")  # Hi Kurran, see here


## Load all required packages
using Distributed

"""
Ensure the desired number of julia processes are present.
"""
function check_procs_IIF(cores::Int)
  if cores > 1
    nprocs() < cores ? addprocs(cores-nprocs()) : nothing
  end
end
check_procs_IIF(4) # make sure there are 4 processes waiting before loading packages

using Dates, Statistics
using Caesar
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


# setup configuration
using YAML

include("configParameters.jl")

# Figure export folder
currdirtime = now()
# currdirtime = "2018-10-28T23:17:30.067"
# currdirtime = "2018-11-03T22:48:51.924"
# currdirtime = "2018-11-07T01:36:52.274"
resultsparentdir = joinpath(datadir, "results")
resultsdir = joinpath(resultsparentdir, "$(currdirtime)")


# When running fresh from new data
include("createResultsDir.jl")



# Utils required for this processing script
include(joinpath(dirname(@__FILE__),"racecarUtils.jl"))
include(joinpath(dirname(@__FILE__),"cameraUtils.jl"))
# include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","visualizationUtils.jl"))



## DETECT APRILTAGS FROM IMAGE DATA
# prep keyframe image data
camlookup = prepCamLookup(camidxs)

# detect tags and extract pose transform
IMGS, TAGS = detectTagsViaCamLookup(camlookup, joinpath(datafolder,imgfolder), resultsdir)

# prep dictionary with all tag detections and poses
tag_bag = prepTagBag(TAGS)

# save the tag detections for later comparison
fid=open(resultsdir*"/tags/pose_tags.csv","w")
for pose in sort(collect(keys(tag_bag)))
  println(fid, "$pose, $(collect(keys(tag_bag[pose])))")
end
close(fid)


# save the tag bag file for future use
@save resultsdir*"/tag_det_per_pose.jld2" tag_bag
# @load resultsdir*"/tag_det_per_pose.jld2" tag_bag



## BUILD FACTOR GRAPH FOR SLAM SOLUTION,

# batch solve after every bb=* poses, use 100 points per marginal
BB = 20
N = 100

# Factor graph construction
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


## sneak peak
# plotKDE(fg, :l1, dims=[3])
# ls(fg)
# drawPosesLandms(fg, spscale=0.25)


# load from previous file
# fg, = loadjld(file=resultsdir*"/racecar_fg_x160.jld2")

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


  if psid % BB == 0 || psid == maxlen
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



results2csv(fg; dir=resultsdir, filename="results.csv")


# # save factor graph for later testing and evaluation
# # fg, = loadjld(file=resultsdir*"/racecar_fg_final.jld2")
# @time tree = batchSolve!(fg, N=N, drawpdf=true, show=true, recursive=true)
#
# IIF.savejld(fg, file=resultsdir*"/racecar_fg_final_resolve.jld2")
# # fgr, = loadjld(file=resultsdir*"/racecar_fg_final_resolve.jld2")
# results2csv(fg; dir=resultsdir, filename="results_resolve.csv")
#
#
# # pl = plotKDE(fg, :x1, levels=1, dims=[1;2]);
#
#
#
# #,xmin=-3,xmax=6,ymin=-5,ymax=2);
# Gadfly.push_theme(:default)
# pl = drawPosesLandms(fg, spscale=0.1, drawhist=false, meanmax=:max);
# # Gadfly.set(:default_theme)
# Gadfly.draw(PNG(joinpath(resultsdir,"images","final.png"),15cm, 10cm),pl);
# Gadfly.draw(SVG(joinpath(resultsdir,"images","final.svg"),15cm, 10cm),pl);
#
# # pl = drawPosesLandms(fg, spscale=0.1, meanmax=:mean) # ,xmin=-3,xmax=3,ymin=-2,ymax=2);
# # Gadfly.draw(PNG(joinpath(resultsdir,"hist_final.png"),15cm, 10cm),pl)
#
# 0




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
