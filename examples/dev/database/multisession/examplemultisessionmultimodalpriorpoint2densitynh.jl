# example of creating multi-session, multi-modal PriorPoint2DensityNH
# addprocs(7)
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
multisessions = ["SESSTURT21";"SESSTURT38";"SESSTURT45"]
multisessions = Vector{String}(setdiff(multisessions, [session]))
addrdict["multisession"] = multisessions

## The major multisession call add the multisession priors to the graph
rmInstMultisessionPriors!(cloudGraph, session=session, multisessions=multisessions)
## and will modify the graph



## Development calls, lower down API
# These are the base steps in rmInstMultisessionPriors!
lm2others = getLandmOtherSessNeoIDs(cloudGraph,
                session=session,multisessions=multisessions)
# grab local subgraph using NeoIDs
sfg, lms = getLocalSubGraphMultisession(cloudGraph, lm2others,
                session=session, numneighbors=1)



sym = lms[6]
neoids = getAllLandmarkNeoIDs(lm2others, sym)
prp2 = getprpt2kde(cloudGraph, neoids, N=100 )

plotKDE(prp2.belief)



# setBackendWorkingSet!(fg.cg.neo4j.connection, session)
# get full graph copy
fg = initfg(sessionname=session, cloudgraph=cloudGraph)
fullLocalGraphCopy!(fg)

# import IncrementalInference: saveplot


ANAITER=1

# for sym in lms
sym = lms[1]
fv = getFactor(fg, sym)
ffv = getFactorType(fv) #getData(fv).fnc.usrfnc!
# plotKDE(ffv.belief)

p = getVariable(fg, sym) |> getBelief
pl = plotKDE([ffv.belief; p], c=["red";"green"], levels=3, legend=["MultiSess"; "Current"], title=string(sym), fill=true)

# run(`mkdir results`)
saveplot(pl, name=string("results/", session,"_",sym, "_$(ANAITER)"), frt=:pdf, nw=true)
saveplot(pl, name=string("results/", session,"_",sym, "_$(ANAITER)"), frt=:png, nw=true)
# end

# vert = getVert(fg, :x30, api=localapi)
# pts = getVal(fg, :x29, api=localapi)
# setVal!(vert, pts)
# updateFullVert!(fg, vert)

lms[7]

plotLocalProduct(fg, lms[7], api=localapi, dims=[1;2])


tree = wipeBuildNewTree!(fg, drawpdf=true)
@async run(`evince bt.pdf`)
inferOverTree!(fg, tree, dbg=true)

spyCliqMat(tree, :x30)

ls(fg, lms[7])

fc = getVert(fg, :x29x30, nt=:fnc, api=localapi)

pts, = findRelatedFromPotential(fg, fc, fg.IDs[:x30], 100)

plotKDE(fg, :x29)
plotKDE(pts,dims=[1;2])

# Juno.breakpoint("/home/dehann/.julia/v0.5/Caesar/src/cloudgraphs/CloudGraphIntegration.jl", 43)
