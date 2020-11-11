#debug fullLocalGraphCopy!
using Caesar, RoME, IncrementalInference
using CloudGraphs, Neo4j
# using Distributions
using JSON

using KernelDensityEstimate

using Base.Test


## Uncomment out for command line operation
# cloudGraph, addrdict = standardcloudgraphsetup(nparticles=true)
# session = addrdict["session"]
# Nparticles = parse(Int,addrdict["num particles"])

# interactive operation
session = "SESSROX"
Nparticles = 100
include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))


fg = initfg(sessionname=session, cloudgraph=cloudGraph)


# setAllDBSolvable!(fg)
setBackendWorkingSet!(session)

fullLocalGraphCopy!(fg)








IDs = getAllExVertexNeoIDs(conn, sessionname=fg.sessionname)

tosee = Dict()
tosee2 = Dict()
for i in 1:length(IDs)
  vid, nid = IDs[i]
  tosee[vid] = nid
  tosee2[nid] = vid
end

tosee2[149088]

writeGraphPdf(fg)
# @async run(`evince fg.pdf`)

tree = wipeBuildNewTree!(fg,drawpdf=true)
# @async run(`evince bt.pdf`)


spyCliqMat(tree, :l200050)


inferOverTreeR!(fg, tree, N=N, dbg=true)


# Some inspection methods to see what is going on
spyCliqMat(tree, :l200050)

plotMCMC(tree, :l200050)

plotPriorsAtCliq(tree, :l200050, :l200050, dims=[1;2], levels=1)

plotUpMsgsAtCliq(tree, :l200050, :l200050, marg=[1;2])


# basic plotting of Pose2D and landmarks
drawPosesLandms(fg ,spscale=0.5)


lbll=:l200050
cliq = getClique(tree, string(lbll))
cliqdbg = cliq.attributes["debug"]
vidx = 1
for lb in cliqdbg.mcmc[1].lbls
  if lb == lbll
    break;
  else
    vidx += 1
  end
end
ppp = kde!(cliqdbg.mcmc[3].prods[vidx].product)
plotKDE(ppp)


vc = startdefaultvisualization(draworigin=true)

drawposepoints!(vc, fg, :x4)

visualizeallposes!(vc, fg, drawlandms=false, drawtype=:fit)



drawPoses(fg,from=4,to=4)

# prior on :x1 issue

fc = getVert(fg, 100007)

vid = fg.IDs[:l200050]
vid = fg.IDs[:x4]


fc

Profile.clear()

@profile pts = evalFactor(fg, fc, vid)
plotKDE(kde!(pts[1:2,:]))


getSample( getData(fc).fnc.usrfnc!, 100 )

plotKDE(getData(fc).fnc.usrfnc!.range)


#
