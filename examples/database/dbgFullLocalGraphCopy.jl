#debug fullLocalGraphCopy!
using Caesar, RoME, IncrementalInference
using CloudGraphs, Neo4j
# using Distributions
using JSON

using KernelDensityEstimate

using Base.Test

# TODO comment out for command line operation
include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
session = "RYPKEMA2"


configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, dbusr, dbpwd, mongoaddress, 27017, false, "", "");
cloudGraph = connect(configuration);
conn = cloudGraph.neo4j.connection
registerGeneralVariableTypes!(cloudGraph)
Caesar.usecloudgraphsdatalayer!()



N=100
fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)


# setDBAllReady!(fg)
# setBackendWorkingSet!(conn, session)

fullLocalGraphCopy!(fg, conn)


# writeGraphPdf(fg)
# @async run(`evince fg.pdf`)

tree = wipeBuildNewTree!(fg,drawpdf=true)
@async run(`evince bt.pdf`)

spyCliqMat(tree, :x1)



inferOverTreeR!(fg, tree, N=N, dbg=true)


# Some inspection methods to see what is going on
spyCliqMat(tree, :x1)

plotMCMC(tree, :x7)

plotPriorsAtCliq(tree, :x1, :x1, marg=[1;2], levels=3)

plotUpMsgsAtCliq(tree, :l200050, :x2, marg=[1;2])


# basic plotting of Pose2D and landmarks
drawPosesLandms(fg ,spscale=0.5)






vc = startdefaultvisualization(draworigin=true)

drawmarginalpoints!(vc, fg, :x8)

visualizeallposes!(vc, fg, drawlandms=false, drawtype=:fit)





# prior on :x1 issue

fc = getVert(fg, 100002)

vid = fg.IDs[:x1]



evalFactor2(fg, fc, vid)


getSample( getData(fc).fnc.usrfnc!, 100 )


getData(fc).fnc.usrfnc!.Cov



#
