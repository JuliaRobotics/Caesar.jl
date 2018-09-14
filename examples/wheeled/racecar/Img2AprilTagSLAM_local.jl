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
# datafolder = ENV["HOME"]*"/data/racecar/labrun5/"
datafolder = ENV["HOME"]*"/data/racecar/labrun6/" # 0:5:1795
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

camidxs = 0:5:1795

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


# Factor graph construction
N = 100
fg = initfg()

psid = 0
pssym = Symbol("x$psid")
# first pose with zero prior
addNode!(fg, pssym, DynPose2(ut=0))
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
    tree = wipeBuildNewTree!(fg, drawpdf=true)
    inferOverTree!(fg,tree, N=N)
  end

  ## save factor graph for later testing and evaluation
  IIF.savejld(fg, file=imgdir*"/racecar_fg_$(psym).jld")
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
batchSolve!(fg, drawpdf=true, N=N)

IIF.savejld(fg, file=imgdir*"/racecar_fg_final_resolve.jld")
# fgr, = loadjld(file=imgdir*"/racecar_fg_final_resolve.jld")
results2csv(fg; dir=imgdir, filename="results_resolve.csv")



#,xmin=-3,xmax=6,ymin=-5,ymax=2);
Gadfly.push_theme(:default)
pl = drawPosesLandms(fg, spscale=0.1, drawhist=false, meanmax=:mean)
# Gadfly.set(:default_theme)
Gadfly.draw(SVG(joinpath(imgdir,"images","final.svg"),15cm, 10cm),pl)
# pl = drawPosesLandms(fg, spscale=0.1, meanmax=:mean) # ,xmin=-3,xmax=3,ymin=-2,ymax=2);
# Gadfly.draw(PNG(joinpath(imgdir,"hist_final.png"),15cm, 10cm),pl)






drawPoses(fg, spscale=1.0)
drawPosesLandms(fg, spscale=0.5)


getAllLandmGifs(fg, IMGS, show=false, dir=imgdir*"/")


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

XX, LL = (KDE.getKDEMax.(IIF.getVertKDE.(fg, IIF.lsf(fg, fsym)))...)
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

#j
# # using KernelDensityEstimate
# # import IncrementalInference: localProduct, proposalbeliefs!, productbelief,findRelatedFromPotential
# function localProduct(fgl::FactorGraph,
#                       sym::Symbol;
#                       N::Int=100,
#                       dbg::Bool=false,
#                       api::DataLayerAPI=IncrementalInference.dlapi  )
#   # TODO -- converge this function with predictbelief for this node
#   # TODO -- update to use getVertId
#   destvertid = fgl.IDs[sym] #destvert.index
#   dens = Array{BallTreeDensity,1}()
#   partials = Dict{Int, Vector{BallTreeDensity}}()
#   lb = String[]
#   fcts = Vector{Graphs.ExVertex}()
#   cf = ls(fgl, sym, api=api)
#   for f in cf
#     vert = getVert(fgl,f, nt=:fnc, api=api)
#     push!(fcts, vert)
#     push!(lb, vert.label)
#   end
#
#   # get proposal beliefs
#   proposalbeliefs!(fgl, destvertid, fcts, dens, partials, N=N, dbg=dbg)
#   # @show length(dens), length(partials)
#
#   # take the product
#   pGM = productbelief(fgl, destvertid, dens, partials, N, dbg=dbg )
#   pp = kde!(pGM)
#
#   return pp, dens, partials, lb
# end
#
#
# function proposalbeliefs!(fgl::FactorGraph,
#       destvertid::Int,
#       factors::Vector{Graphs.ExVertex},
#       dens::Vector{BallTreeDensity},
#       partials::Dict{Int, Vector{BallTreeDensity}};
#       N::Int=100,
#       dbg::Bool=false )
#   #
#   for fct in factors
#     data = getData(fct)
#     p = findRelatedFromPotential(fgl, fct, destvertid, N, dbg)
#     if data.fnc.partial   # partial density
#       pardims = data.fnc.usrfnc!.partial
#       for dimnum in pardims
#         if haskey(partials, dimnum)
#           push!(partials[dimnum], marginal(p,[dimnum]))
#         else
#           partials[dimnum] = BallTreeDensity[marginal(p,[dimnum])]
#         end
#       end
#     else # full density
#       push!(dens, p)
#     end
#   end
#   nothing
# end
#







0
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









0



#
