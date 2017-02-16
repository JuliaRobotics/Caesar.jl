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

setDBAllReady!(fg)
setBackendWorkingSet!(conn, session)

fullLocalGraphCopy!(fg, conn)

incry = fg.g.inclist

@show ls(fg, :l200050)


writeGraphPdf(fg)
run(`evince fg.pdf`)




using Graphs

fgd = IncrementalInference.drawCopyFG(fg)
tt = fgd.g.vertices[100004]
for vv in out_neighbors(tt, fgd.g)
  @show vv.label
end





tree = wipeBuildNewTree!(fg,drawpdf=true)
@async run(`evince bt.pdf`)

spyCliqMat(tree, :x2)


fc = getVert(fg, 100004)

vid = fg.IDs[:l200050]

evalFactor2(fg, fc, vid)

inferOverTreeR!(fg, tree, N=N, dbg=true)

plotMCMC(tree, :l200050)

ls(fg)



cliq = whichCliq(tree,:l200050)
cliqdbg = cliq.attributes["debug"]
vidx = 1

@show cliqdbg.mcmc

for lb in cliqdbg.mcmc[1].lbls
  if lb == lbll
    break;
  else
    vidx += 1
  end
end






















#
