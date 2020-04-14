# quick script to draw Pose2D what is in the DB

using Caesar, RoME, Gadfly


## Uncomment out for command line operation
cloudGraph, addrdict = standardcloudgraphsetup(nparticles=true)
session = addrdict["session"]
Nparticles = parse(Int,addrdict["num particles"])

# interactive operation
# session = "SESSROX"
# Nparticles = 100
# include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))


# fieldnames(fg.cg.neo4j)

fg = initfg(sessionname=session, cloudgraph=cloudGraph)

# setBackendWorkingSet!(conn, session)
fullLocalGraphCopy!(fg)

println("Plotting...")
pl = drawPosesLandms(fg)

println("Rendering plotpose2_$(session).pdf...")
draw(PDF("plotpose2_$(session).pdf",40cm,40cm),pl)

#
