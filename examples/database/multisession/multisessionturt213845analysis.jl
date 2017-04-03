# Analyse multisession solution progression

using Caesar
# IIF and KDE should not be necessary, but adding for late night run safety
using IncrementalInference
using KernelDensityEstimate

# get credentials
addrdict = getcredentials(nparticles=true, multisession=true)

# setup cloudgraphs interface
cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)

# extras
multisessions = addrdict["multisession"]
N = parse(Int,addrdict["num particles"])

# RESET ENTIRE SYSTEM
removeMultisessions!(cloudGraph, session=multisessions[1])
removeMultisessions!(cloudGraph, session=multisessions[2])
removeMultisessions!(cloudGraph, session=multisessions[3])

addrdict["session"] = multisessions[1]
resetconvertdb(addrdict=addrdict)
setDBAllReady!(cloudGraph.neo4j.connection, addrdict["session"])
convertdb(addrdict=addrdict, N=N)
addrdict["session"] = multisessions[2]
resetconvertdb(addrdict=addrdict)
setDBAllReady!(cloudGraph.neo4j.connection, addrdict["session"])
convertdb(addrdict=addrdict, N=N)
addrdict["session"] = multisessions[3]
resetconvertdb(addrdict=addrdict)
setDBAllReady!(cloudGraph.neo4j.connection, addrdict["session"])
convertdb(addrdict=addrdict, N=N)


resultsfolder = "results_$(Dates.now(Dates.UTC))"
run(`mkdir -p $(resultsfolder)`)

for ANAITER in Int[1;2;3;4;5]
  for session in multisessions
    @show addrdict["session"] = session
    slamindb(addrdict=addrdict, iterations=1)

    # These are the base steps in rmInstMultisessionPriors!
    lm2others = getLandmOtherSessNeoIDs(cloudGraph,
                    session=session,multisessions=multisessions)
    # grab local subgraph using NeoIDs
    sfg, lms = getLocalSubGraphMultisession(cloudGraph, lm2others,
                    session=session, numneighbors=1)

    fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)
    fullLocalGraphCopy!(fg)

    jldfile = string(resultsfolder,"/", session, "_$(ANAITER).jld")
    savejld(fg, file=jldfile)
    for sym in lms
      fv = getVert(fg, sym, nt=:fnc)
      ffv = getData(fv).fnc.usrfnc!
      # plotKDE(ffv.belief)

      p = getVertKDE(fg, sym)
      pl = plotKDE([ffv.belief; p], c=["red";"green"], levels=3, legend=["MultiSess"; "Current"], title=string(sym), fill=true)

      filename = string(resultsfolder,"/", session,"_",sym, "_$(ANAITER)")
      saveplot(pl, name=filename, frt=:pdf, nw=true)
      saveplot(pl, name=filename, frt=:png, nw=true)
    end
  end
end
