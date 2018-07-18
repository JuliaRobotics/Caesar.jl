# add more julia processes
nprocs() < 4 ? addprocs(4-nprocs()) : nothing

using IncrementalInference
using HDF5, JLD, Gadfly, Colors, Cairo
using KernelDensityEstimate, Distributions
using Caesar, RoME
using RoMEPlotting
using DocStringExtensions
using ArgParse

## load all the model data
# d = odometry information
# f = laser scanner detections
# MM = multi-modal individual id references
# MMr = reworked to map to only one previous feature
# examplefolder, datafolder

include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","loadVicPrkData.jl"))


# include(joinpath(ENV["HOME"],"Documents","blandauthlocal.jl"))
# backend_config, user_config = standardcloudgraphsetup(addrdict=addrdict)
# # Start new session
# addrdict["sessionId"] = "VICPRK_VID"
# addrdict["robotId"] = "Ute"
# fg = Caesar.initfg(sessionname=user_config["sessionId"], robotname=user_config["robotId"], cloudgraph=backend_config)
# deleteServerSession!(fg.cg, user_config["sessionId"])


include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","VicPrk_FrontEndUtils.jl"))




# run using local inference
function mainrun(dd,ff; frames=3)

    currdirtime = now()
    imgdir = joinpath(ENV["HOME"], "Pictures", "vicprkimgs", "$(currdirtime)")
    mkdir(imgdir)

    ddd = deepcopy(dd)
    fff = deepcopy(ff)

    PL = Dict{Int, Any}()

    Podo=diagm([0.5;0.5;0.005])
    lsrNoise=diagm([0.1;1.0])

    # start factor graph
    N=75
    fg = Caesar.initfg()
    lmoccur = Dict{Symbol, Int}()

    # init pose
    prevn = initFactorGraph!(fg, init=ddd[1][1:3])[1]

    findAddNewLandmarks!(fg, lmoccur, fff[1])
    # there might not be any landmarks yet
    findAddLandmFactorsByPose!(fg, :x0, fff[1], N=N)
    tree = wipeBuildNewTree!(fg, drawpdf=true)
    inferOverTree!(fg, tree, N=N)
    PL[1] = drawPoses(fg)

    for i in 2:frames
      PL[i] = donextframe!(fg, i, ddd, fff, lmoccur, Podo, N=N)
      Gadfly.draw(PDF(imgdir*"/x$(i).pdf", 30cm,20cm),PL[i])
      Gadfly.draw(PNG(imgdir*"/x$(i).png", 30cm,20cm),PL[i])
    end
    PL
end




# @async run('evince bt.pdf')
PL = mainrun(d,f; frames=10)



#
