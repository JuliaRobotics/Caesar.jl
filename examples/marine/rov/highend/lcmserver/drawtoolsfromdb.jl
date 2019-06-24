# template log-on and fetch graph

using Caesar


include(joinpath(dirname(@__FILE__),"..","database","blandauthremote.jl"))
addrdict["session"] = "SESS??"
addrdict["robot"] = "robot"
addrdict["user"] = "user"
cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)


fg = initfg(cloudgraph=cloudGraph, sessionname=addrdict["session"])
fullLocalGraphCopy!(fg)
# savejld(fg, file="somefg.jld")


ls(fg)

vis = startdefaultvisualization(draworigin=true)

visualizeDensityMesh!(vis, fg, :x4, levels=5)


plotKDE(fg, :x1, dims=[1;2], api=localapi)



pl = plotKDE(fg, :x1, api=localapi)
using Gadfly
Gadfly.draw(PDF("test.pdf",40cm, 40cm),pl)
@async run(`evince test.pdf`)




using IncrementalInference

fncvar = getfnctype(getVert(fg,fg.fIDs[:x1710l200050]))
#TODO: Please fix with correct session for example
ret = whosNear2D(cloudGraph, "SESS??", "robot", "user", x=9.8, y=15.2, dist=0.5)
plotLocalProduct(fg, :x1, dims=[1;2], api=localapi)

plotKDEresiduals(fg, :x1x2, dims=[1;2], api=localapi)

pts = getVal(fg, :x5, api=localapi)



using Distributions

fit(MvNormal, pts)






#
