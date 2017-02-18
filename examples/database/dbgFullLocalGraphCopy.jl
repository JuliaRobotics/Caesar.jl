#debug fullLocalGraphCopy!
using Caesar, RoME, IncrementalInference
using CloudGraphs, Neo4j
# using Distributions
using JSON

using KernelDensityEstimate

using Base.Test

# TODO comment out for command line operation
include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
session = "SANDSHARK_2016_11_14"


configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, dbusr, dbpwd, mongoaddress, 27017, false, "", "");
cloudGraph = connect(configuration);
conn = cloudGraph.neo4j.connection
registerGeneralVariableTypes!(cloudGraph)
Caesar.usecloudgraphsdatalayer!()


N=100
fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)


# setDBAllReady!(fg)
setBackendWorkingSet!(conn, session)

fullLocalGraphCopy!(fg, conn)








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

plotPriorsAtCliq(tree, :l200050, :l200050, marg=[1;2], levels=1)

plotUpMsgsAtCliq(tree, :l200050, :l200050, marg=[1;2])


# basic plotting of Pose2D and landmarks
drawPosesLandms(fg ,spscale=0.5)


lbll=:l200050
cliq = whichCliq(tree, string(lbll))
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

drawmarginalpoints!(vc, fg, :x4)

visualizeallposes!(vc, fg, drawlandms=false, drawtype=:fit)



drawPoses(fg,from=4,to=4)

# prior on :x1 issue

fc = getVert(fg, 100007)

vid = fg.IDs[:l200050]
vid = fg.IDs[:x4]


fc

Profile.clear()

@profile pts = evalFactor2(fg, fc, vid)
plotKDE(kde!(pts[1:2,:]))


getSample( getData(fc).fnc.usrfnc!, 100 )

plotKDE(getData(fc).fnc.usrfnc!.range)


#
