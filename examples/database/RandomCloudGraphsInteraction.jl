# random DB interactions


include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
session = "SESSROX"

# Connect to database
configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, dbusr, dbpwd, mongoaddress, 27017, false, "", "");
cloudGraph = connect(configuration);
conn = cloudGraph.neo4j.connection
# register types of interest in CloudGraphs
registerGeneralVariableTypes!(cloudGraph)
Caesar.usecloudgraphsdatalayer!()




fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)
fullLocalGraphCopy!(fg, conn)

@show usrf = getData(fg.g.vertices[100003]).fnc.usrfnc!
pts = evalFactor2(fg, fg.g.vertices[100003], fg.IDs[:l15])

pts = evalFactor2(fg, fg.g.vertices[100003], fg.IDs[:x4])


v = getVert(fg, :x4, api=dlapi)
pts = getVal(fg, :x4, api=dlapi)

@show usrf.range

pts = getSample(usrf)
res = zeros(2)
usrf(res, 1, pts, getVal(fg, :x4), getVal(fg, :l15))
@show res

@show fieldnames(getData(fg.g.vertices[100003]).fnc)
@show getData(fg.g.vertices[100003]).fnc.particleidx

v = getVert(fg, :l15, api=dlapi)
getVal(fg, :l15, api=dlapi)

setVal!(v, randn(2,100))
updateFullCloudVertData!(fg, v)
