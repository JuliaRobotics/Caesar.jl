# foveation example

using Caesar
# using Neo4j
# using CloudGraphs

addrdict = nothing
include(joinpath(dirname(@__FILE__),"..","database","blandauthremote.jl"))
# prepare the factor graph with just one node
# (will prompt on stdin for db credentials)
cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)



#TODO: Please replace with correct robot and user name.
neoids, syms = foveateQueryToPoint(cloudGraph,
        ["SESSTURT21";"SESSTURT38";"SESSTURT45"], "robot", "user"
        point=[-9.0;9.0], fovrad=0.5)




# @async cloudimshow(cloudGraph, neoid=neoids[1])
for neoid in neoids
  cloudimshow(cloudGraph, neoid=neoid)
end















#
