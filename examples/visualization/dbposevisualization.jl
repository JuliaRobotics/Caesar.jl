# test draw pose

using Caesar, CloudGraphs, IncrementalInference

using DrakeVisualizer

vc = startdefaultvisualization()


# include(joinpath(dirname(@__FILE__),"..","database","blandauthremote.jl"))
# addrdict["session"] = "SESSTURT21"
# cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)
cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)


session = ddrdict["session"]

# fg = Caesar.initfg(sessionname=addrdict["session"], cloudgraph=cloudGraph)

IDs = getPoseExVertexNeoIDs(cloudGraph.neo4j.connection, session=addrdict["session"], reqbackendset=false);


cv = CloudGraphs.get_vertex(cloudGraph, IDs[10][2])
vert = cloudVertex2ExVertex(cv)


drawpose!(vc, vert, session=session, drawtype=:max )
drawposepoints!(vc, vert, session=session )



















#
