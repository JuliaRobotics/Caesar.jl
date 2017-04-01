# example of creating multi-session, multi-modal PriorPoint2DensityNH

using Caesar


N = 100

# standard setup, auth from file
include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
addrdict["num particles"] = "$(N)"
addrdict["session"] = "SESSTURT21"
cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)

# convenience variables
# conn = cloudGraph.neo4j.connection


session = addrdict["session"]
multisessions = ["SESSTURT38";"SESSTURT45"]

multisessionsl = Vector{String}(setdiff(multisessions, [session]))


# add the multisession priors to the graph
rmInstMultisessionPriors!(cloudGraph, session=session, multisessions=multisessions)




# These are the base steps in rmInstMultisessionPriors!
  lm2others = getLandmOtherSessNeoIDs(cloudGraph,
                  session=session,multisessions=multisessions)
  # grab local subgraph using NeoIDs
  sfg, lms = getLocalSubGraphMultisession(cloudGraph, lm2others,
                  session=session, numneighbors=2)
  sym = lms[1]
  neoids = getAllLandmarkNeoIDs(lm2others, sym)
  prp2 = getprpt2kde(cloudGraph, neoids, N=100 )
  plotKDE(prp2.belief)









# Juno.breakpoint("/home/dehann/.julia/v0.5/Caesar/src/cloudgraphs/CloudGraphIntegration.jl", 43)
