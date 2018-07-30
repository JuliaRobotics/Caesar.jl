    # Local compute version

# add more julia processes
nprocs() < 5 ? addprocs(5-nprocs()) : nothing

using Caesar, RoME, Distributions
using YAML, JLD, HDF5
# for drawing
using RoMEPlotting, Gadfly

# const IIF = IncrementalInference

include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","racecarUtils.jl"))

# function loadConfig()
#   cfg = Dict{Symbol,Any}()
#   data =   YAML.load(open(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","cam_cal.yml")))
#   bRc = eval(parse("["*data["extrinsics"]["bRc"][1]*"]"))
#   cfg[:bRc] = bRc
#   cfg[:intrinsics] = Dict{Symbol,Any}()
#   cfg[:intrinsics][:height] = data["intrinsics"]["height"]
#   cfg[:intrinsics][:width] = data["intrinsics"]["width"]
#   cfg[:intrinsics][:cam_matrix] = data["intrinsics"]["camera_matrix"]
#   cfg
# end


cfg = loadConfig()

# datafolder = ENV["HOME"]*"/data/racecar/smallloop/"
datafolder = ENV["HOME"]*"/data/racecar/thursday/"
@load datafolder*"tag_det_per_pose.jld" tag_bag
# tag_bag[0]


# Figure export folder
currdirtime = now()
imgdir = joinpath(ENV["HOME"], "Pictures", "racecarimgs", "$(currdirtime)")
mkdir(imgdir)


# Factor graph construction
N = 75
fg = initfg()

psid = 0
pssym = Symbol("x$psid")
# first pose with zero prior
addNode!(fg, pssym, Pose2)
addFactor!(fg, [pssym], PriorPose2(MvNormal(zeros(3),diagm([0.01;0.01;0.001].^2))))

addApriltags!(fg, pssym, tag_bag[psid])

writeGraphPdf(fg)

# quick solve as sanity check
tree = wipeBuildNewTree!(fg, drawpdf=true)
inferOverTree!(fg,tree, N=N)

#
# plotKDE(getVertKDE(fg, :x0), dims=[1;2])
# plotKDE(getVertKDE(fg, :x0), dims=[1;2])

delete!(tag_bag[61], 18)
delete!(tag_bag[77], 18)
delete!(tag_bag[86], 16)
delete!(tag_bag[110], 6)
delete!(tag_bag[123], 0)
delete!(tag_bag[124], 16)


@async run(`evince bt.pdf`)

prev_psid = 0
# add other positions
for psid in 1:1:115 #length(tag_bag)
  addnextpose!(fg, prev_psid, psid, tag_bag[psid])
  # writeGraphPdf(fg)
  if psid % 25 == 0 || psid == 115
    tree = wipeBuildNewTree!(fg, drawpdf=true)
    inferOverTree!(fg,tree, N=N)

    pl = drawPosesLandms(fg, spscale=0.1, drawhist=false,   meanmax=:mean,xmin=-3,xmax=6,ymin=-5,ymax=2);
    Gadfly.draw(PNG(joinpath(imgdir,"x$(psid).png"),30cm, 25cm),pl)
    pl = drawPosesLandms(fg, spscale=0.1,   meanmax=:mean,xmin=-3,xmax=3,ymin=-2,ymax=2);
    Gadfly.draw(PNG(joinpath(imgdir,"hist_x$(psid).png"),30cm, 25cm),pl)
  end
  prev_psid = psid
end

IncrementalInference.savejld(fg, file="racecar_fg_$(currdirtime).jld")

# fg, = IncrementalInference.loadjld(file="30jul18_9AM.jld")
# addFactor!(fg, [:l5;:l16], Point2Point2Range(Normal(0.5,1.0)))
# ls(fg,:l5)

const KDE = KernelDensityEstimate

fid = open("results.csv","w")
for sym in [ls(fg)[1]...;ls(fg)[2]...]
  p = getVertKDE(fg, sym)
  val = string(KDE.getKDEMax(p))
  println(fid, "$sym, $(val[2:(end-1)])")
end
close(fid)




0

# debugging
#
# using IncrementalInference
# isInitialized(fg,:x0)
# IncrementalInference.doautoinit!(fg, :x0, N=N)
#
# x0l0 = ls(fg, :l0)[1]
# approxConv(fg, x0l0, :l0)
#
#
#
# fct = getVert(fg, x0l0, nt=:fnc)
# ppbr = getfnctype(fct)
#
# X0 = getVal(fg, :x0)
# L0 = getVal(fg, :l0)
#
#
#
#
#
#
#
# IncrementalInference.doautoinit!(fg, :l0, N=N)
# # IncrementalInference.doautoinit!(fg, :l1, N=N)
#
#
# isInitialized(fg, :l0)
#
#
# X1 = getVal(fg,:x0)
#
# plot(x=X1[1,:],y=X1[2,:], Geom.hexbin)
#

#


#
