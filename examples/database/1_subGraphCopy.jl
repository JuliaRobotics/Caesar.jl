## test subLocalGRaphCopy! function of cloudgraphs

# 0. Load required packages
using Caesar


# 1. Get the authentication and session information
include(joinpath(ENV["HOME"],"Documents","blandauthlocal.jl"))
cg, addrdict = standardcloudgraphsetup(addrdict=addrdict)
addrdict["sessionId"] = "Hackathon"
addrdict["robotId"] = "PixieBot"


# 2. Create a local empty factor graph object
fg = initfg(sessionname=addrdict["sessionId"], robotname=addrdict["robotId"], cloudgraph=cg)
# fg.stateless = true # so that we don't update local elements


# 3.a Fetch a portion of the graph
Caesar.subLocalGraphCopy!(fg, ["x1";"x3";], neighbors=1, reqbackendset=false, reqSolvable=false)

# see what is going on
Graphs.plot(fg.g)



# ===========ALTERNATIVE========================================================

fg = initfg(sessionname=addrdict["sessionId"], robotname=addrdict["robotId"], cloudgraph=cg)
# 3.b Or fetch the entire factor graph
Caesar.fullLocalGraphCopy!(fg, reqbackendset=false, reqSolvable=false)




#
