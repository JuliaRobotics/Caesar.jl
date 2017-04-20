# template log-on and fetch graph

using Caesar


include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
session = "SESSHAUVDEV2"
addrdict["session"] = session
cg, addrdict = standardcloudgraphsetup(addrdict=addrdict)





dd = ls(cg, session, sym=:x1)

dd2 = ls(cg, session, exvid=5)





#
