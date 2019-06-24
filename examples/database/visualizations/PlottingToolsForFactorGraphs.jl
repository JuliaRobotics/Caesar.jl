# template log-on and fetch graph

using Caesar
using IncrementalInference

include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
addrdict["session"] = "SESSSHARK_16_11_14"  # "SESSROX"
addrdict["robot"] = "robot"
addrdict["user"] = "user"
cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)


fg = initfg(cloudgraph=cloudGraph, sessionname=addrdict["session"])
fullLocalGraphCopy!(fg)
savejld(fg, file="sandshark_testing_seg417_iNA.jld")




plotKDE(fg, :x1710, dims=[1;2], api=localapi)

plotKDE(fg, :x1723, dims=[1;2], api=localapi)
plotKDE(fg, :x1724, dims=[3], api=localapi)
plotKDE(fg, :x1725, dims=[3], api=localapi)

plotKDE(fg, :x1662, dims=[1;2], api=localapi)

plotKDE(fg, :l200050, dims=[1;2], api=localapi)
plotLocalProduct(fg, :l200050, dims=[1;2], api=localapi)


# pp, = localProduct(fg, :x1710, api=localapi)

plotLocalProduct(fg, :x1710, dims=[1;2], api=localapi)
plotLocalProduct(fg, :x1723, dims=[1;2], api=localapi, levels=1)
pl = plotLocalProduct(fg, :x1724, dims=[1;2], api=localapi, levels=1)


using KernelDensityEstimate
pl = plotKDEresiduals(fg, :x1710l200050, fill=true)

fncvar = getfnctype(getVert(fg,fg.fIDs[:x1710l200050]))

using Gadfly, Colors

Gadfly.draw(PDF("residual_x1710l200050_NA.pdf",15cm,10cm),pl)

h1 = plotKDE(fncvar.range, title=string("Likelihood density estimate, $(:x1710l200050)\n", fncvar), xlbl="Range [m]", fill=true )
h2 = plotKDE(fncvar.bearing, xlbl="Bearing [rad]", fill=true )

Gadfly.draw(PDF("factor_x1710l200050_stack.pdf",15cm,20cm),vstack(h1,h2))


# TODO: Please fix with correct session for example.
ret = whosNear2D(cloudGraph, "SESSSHARK_16_11_14", "robot", "user" x=21.7, y=-54.0)


sort(collect(keys(ret)))


#
