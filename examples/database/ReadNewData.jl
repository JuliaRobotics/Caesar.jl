
# find and interact with NEWDATA in neo4j


using Caesar
using RoME, IncrementalInference
using CloudGraphs, Neo4j
# using Distributions
using JSON


# TODO comment out for command line operation
include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
session = "SESSROX"

# # # connect to the server, CloudGraph stuff
# dbaddress = length(ARGS) > 0 ? ARGS[1] : "localhost"
# println("Taking Neo4j database address as $(dbaddress)...")
#
# dbusr = length(ARGS) > 1 ? ARGS[2] : ""
# dbpwd = length(ARGS) > 2 ? ARGS[3] : ""
#
# mongoaddress = length(ARGS) > 3 ? ARGS[4] : "localhost"
# println("Taking Mongo database address as $(mongoaddress)...")
#
# session = length(ARGS) > 4 ? string(ARGS[5]) : ""
# println("Attempting to draw session $(session)...")
#
# DRAWDEPTH = length(ARGS) > 5 ? ARGS[6]=="drawdepth" : false


# Connection to database
# register types of interest in CloudGraphs
configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, dbusr, dbpwd, mongoaddress, 27017, false, "", "");
cloudGraph = connect(configuration);
conn = cloudGraph.neo4j.connection
registerGeneralVariableTypes!(cloudGraph)
Caesar.usecloudgraphsdatalayer!()

# fieldnames(fg.cg.neo4j)

N=100
fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)

# setBackendWorkingSet!(conn, session)
fullLocalGraphCopy!(fg, conn)

# @show length(fg.IDs)
# showcurrentdlapi()

updatenewverts!(fg, N=N)

# getVal(fg, :x6)

@show length(fg.IDs)



writeGraphPdf(fg)
@async run(`evince fg.pdf`)



# Okay now solve locally, or start MM-iSAMCloudSolve.jl
fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)

setDBAllReady!(fg)
setBackendWorkingSet!(conn, session)
fullLocalGraphCopy!(fg, conn)

tree = wipeBuildNewTree!(fg,drawpdf=true)
@async run(`evince bt.pdf`)


# single core recursive inference, allowing better stacktrace
inferOverTreeR!(fg, tree, N=N)



# totally reset to frontend built state in DB
resetentireremotesession(conn,session)



#
