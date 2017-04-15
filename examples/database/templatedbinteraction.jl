# template log-on and fetch graph

using Caesar
using IncrementalInference

include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
addrdict["session"] = "SESS??"
cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)


fg = Caesar.initfg(cloudgraph=cloudGraph, sessionname=addrdict["session"])
fullLocalGraphCopy!(fg)
savejld(fg, file="somefg.jld")



plotKDE(fg, :x1, dims=[1;2], api=localapi)
fncvar = getfnctype(getVert(fg,fg.fIDs[:x1710l200050]))
ret = whosNear2D(cloudGraph, "SESS??", x=21.7, y=-54.0)
plotLocalProduct(fg, :l200050, dims=[1;2], api=localapi)










#
