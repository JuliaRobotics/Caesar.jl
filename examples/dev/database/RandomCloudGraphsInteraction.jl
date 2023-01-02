# random DB interactions


## Uncomment out for command line operation
# cloudGraph, addrdict = standardcloudgraphsetup(nparticles=true)
# session = addrdict["session"]
# Nparticles = parse(Int,addrdict["num particles"])

# interactive operation
session = "SESSROX"
Nparticles = 100
include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))





fg = initfg(sessionname=session, cloudgraph=cloudGraph)
fullLocalGraphCopy!(fg)

@show usrf = getData(fg.g.vertices[100003]).fnc.usrfnc!
pts = evalFactor(fg, fg.g.vertices[100003], fg.IDs[:l15])

pts = evalFactor(fg, fg.g.vertices[100003], fg.IDs[:x4])


v = getVert(fg, :x4, api=dlapi)
pts = getVal(fg, :x4, api=dlapi)

@show usrf.range

pts = getSample(usrf)
res = zeros(2)
usrf(res, 1, pts, getVal(fg, :x4), getVal(fg, :l15))
@show res

@show fieldnames(getData(fg.g.vertices[100003]).fnc)
@show getData(fg.g.vertices[100003]).fnc.particleidx[]

v = getVert(fg, :l15, api=dlapi)
getVal(fg, :l15, api=dlapi)

setVal!(v, randn(2,100))
updateFullCloudVertData!(fg, v)
