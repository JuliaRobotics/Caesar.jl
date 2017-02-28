# convert uploaded graph in DB to MM-iSAM ready form

using Caesar, CloudGraphs


# Uncomment out for command line operation
cloudGraph, addrdict = standardcloudgraphsetup(nparticles=true, clearslamindb=true)
session = addrdict["session"]
@show clearslamindbdata = addrdict["clearslamindb"]=="y" || addrdict["clearslamindb"]=="yes"

# interactive operation
# session = "SESSROX"
# Nparticles = 100
# include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))



if !clearslamindbdata
  fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)

  updatenewverts!(fg, N=Nparticles)
else
  println("Clearing slamindb data, leaving front-end data, session: $(session)")
  resetentireremotesession(conn,session)
end


println("Done.")
