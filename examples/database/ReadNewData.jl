# find and interact with NEWDATA in neo4j
using Caesar


using RoME, IncrementalInference
using CloudGraphs, Neo4j
# using Distributions
using JSON


## Uncomment out for command line operation
# cloudGraph, addrdict = standardcloudgraphsetup(nparticles=true)
# session = addrdict["session"]
# Nparticles = parse(Int,addrdict["num particles"])

# interactive operation
session = "SESSROX"
Nparticles = 100
include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))



# fieldnames(fg.cg.neo4j)

N=Nparticles
fg = initfg(sessionname=session, cloudgraph=cloudGraph)

# setBackendWorkingSet!(conn, session)
fullLocalGraphCopy!(fg)

# @show length(fg.IDs)
# showcurrentdlapi()

updatenewverts!(fg, N=N)

# getVal(fg, :x6)

@show length(fg.IDs)



writeGraphPdf(fg)
@async run(`evince fg.pdf`)



# Okay now solve locally, or start MM-iSAMCloudSolve.jl
fg = initfg(sessionname=session, cloudgraph=cloudGraph)

setAllDBSolvable!(fg)
setBackendWorkingSet!(conn, session)
fullLocalGraphCopy!(fg)

tree = wipeBuildNewTree!(fg,drawpdf=true)
@async run(`evince bt.pdf`)


# single core recursive inference, allowing better stacktrace
inferOverTreeR!(fg, tree, N=N)



# totally reset to frontend built state in DB
resetentireremotesession(fg.cg.neo4j.connection, session)



#
