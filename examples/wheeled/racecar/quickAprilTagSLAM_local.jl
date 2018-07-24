# Local compute version

# add more julia processes
nprocs() < 4 ? addprocs(4-nprocs()) : nothing

using Caesar, Distributions
using YAML, JLD, HDF5
# for drawing
using RoMEPlotting, Gadfly



function loadConfig()
  cfg = Dict{Symbol,Any}()
  data =   YAML.load(open(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","cam_cal.yml")))
  bRc = eval(parse("["*data["extrinsics"]["bRc"][1]*"]"))
  cfg[:bRc] = bRc
  cfg[:intrinsics] = Dict{Symbol,Any}()
  cfg[:intrinsics][:height] = data["intrinsics"]["height"]
  cfg[:intrinsics][:width] = data["intrinsics"]["width"]
  cfg[:intrinsics][:cam_matrix] = data["intrinsics"]["camera_matrix"]
  cfg
end

# add AprilTag sightings from this pose
function addApriltags!(fg, pssym, posetags; bnoise=0.1, rnoise=0.3 )
  currtags = ls(fg)[2]
  for lmid in keys(posetags)
    @show lmsym = Symbol("l$lmid")
    if !(lmsym in currtags)
      addNode!(fg, lmsym, Point2)
    end
    ppbr = Pose2Point2BearingRange(Normal(posetags[lmid][:bearing][1],bnoise),
                                   Normal(posetags[lmid][:range][1],rnoise))
    addFactor!(fg, [pssym;lmsym], ppbr, autoinit=false)
  end
  nothing
end

function addnextpose!(fg, prev_psid, new_psid, pose_tag_bag)
  prev_pssym = Symbol("x$(prev_psid)")
  new_pssym = Symbol("x$(new_psid)")
  # first pose with zero prior
  addNode!(fg, new_pssym, Pose2)
  addFactor!(fg, [prev_pssym; new_pssym], Pose2Pose2(MvNormal(zeros(3),diagm([0.5;0.5;0.3].^2))))

  addApriltags!(fg, new_pssym, pose_tag_bag)
  new_pssym
end


cfg = loadConfig()

datafolder = ENV["HOME"]*"/data/racecar/smallloop/"
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
#
# drawPosesLandms(fg)
# plotKDE(getVertKDE(fg, :x0), dims=[1;2])
# plotKDE(getVertKDE(fg, :x0), dims=[1;2])

@async run(`evince bt.pdf`)

prev_psid = 0
# add other positions
for psid in 1:1:59
  addnextpose!(fg, prev_psid, psid, tag_bag[psid])
  # writeGraphPdf(fg)
  tree = wipeBuildNewTree!(fg, drawpdf=true)
  inferOverTree!(fg,tree, N=N)

  pl = drawPosesLandms(fg, spscale=0.1, drawhist=false, meanmax=:mean,xmin=-3,xmax=6,ymin=-5,ymax=2);
  Gadfly.draw(PNG(joinpath(imgdir,"x$(psid).png"),30cm, 25cm),pl)
  pl = drawPosesLandms(fg, spscale=0.1, meanmax=:mean,xmin=-3,xmax=6,ymin=-5,ymax=2);
  Gadfly.draw(PNG(joinpath(imgdir,"hist_x$(psid).png"),30cm, 25cm),pl)
  prev_psid = psid
end



for i in 1:3
tree = wipeBuildNewTree!(fg, drawpdf=true)
inferOverTree!(fg,tree, N=N)
end

drawPosesLandms(fg,spscale=0.1, drawhist=false,xmin=-3,xmax=6,ymin=-5,ymax=2)



# add second position
psid = 20
addnextpose!(fg, 10, psid, tag_bag[psid])

writeGraphPdf(fg)


tree = wipeBuildNewTree!(fg)
inferOverTree!(fg,tree, N=N)


drawPosesLandms(fg)





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
