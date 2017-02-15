# lets see if we can interpret the data

using Caesar, RoME, IncrementalInference
using CloudGraphs, Neo4j
# using Distributions
using JSON

using KernelDensityEstimate

using Base.Test

# TODO comment out for command line operation
include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
session = "RYPKEMA"


configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, dbusr, dbpwd, mongoaddress, 27017, false, "", "");
cloudGraph = connect(configuration);
conn = cloudGraph.neo4j.connection
registerGeneralVariableTypes!(cloudGraph)
Caesar.usecloudgraphsdatalayer!()


N=100
fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)


updatenewverts!(fg, N=N)


fg.fIDs

# sortedvd = getnewvertdict(fg.cg.neo4j.connection, fg.sessionname)
#
# sortedvd[110240]
#
# elem = sortedvd[110536][:frtend]
#
# elem["range"]
#
# rn = recoverConstraintType(elem)

# pty = convert(PackedPose2DPoint2DBearingRangeDensity, rn)
# upt = convert(Pose2DPoint2DBearingRangeDensity, pty)

# @test norm(getPoints(rn.bearing)-getPoints(upt.bearing)) < 1e-10
# @test norm(getPoints(rn.range)-getPoints(upt.range)) < 1e-10

# plotKDE(upt.bearing)
# plotKDE(rn.range)


vc = startdefaultvisualization(draworigin=true)

drawmarginalpoints!(vc, fg, :l200050)

getVal(fg, :x1)

visualizeallposes!(vc, fg, drawlandms=false)

# for lb in [:x1,:x2,:x3,:x4,:x5,:x6,:x7]
#   @show getKDEMax(getVertKDE(fg, lb))
# end
plotKDE(marginal(getKDE(getVert(fg,:l200050,api=dlapi)),[1;2]))


fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)

setBackendWorkingSet!(conn, session)
fullLocalGraphCopy!(fg, conn)

tree = wipeBuildNewTree!(fg,drawpdf=false)
@async run(`evince bt.pdf`)

spyCliqMat(tree, :x1)

ls(fg,:l200050)

inferOverTree!(fg, tree, N=N)


ls(fg)


writeGraphPdf(fg)
run(`evince fg.pdf`)

fg.IDs


import IncrementalInference: drawCopyFG, writeGraphPdf




#
