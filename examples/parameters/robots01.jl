# example for loading robot description to database

using Caesar
using JSON
using Neo4j
using CloudGraphs

addrdict = nothing
include(joinpath(dirname(@__FILE__),"..","database","blandauthremote.jl"))
# prepare the factor graph with just one node
# (will prompt on stdin for db credentials)
cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)



# Turtlebot robot
turtlebot = Dict()
turtlebot["robot"] = "mrg-turtlebot"
turtlebot["CAMK"] = [ 570.34222412 0.0 319.5;
                      0.0 570.34222412 239.5;
                      0.0 0.0 1.0]
#
turtlebot["imshape"]=[480,640]
turtlebot["bTc"] = [0.0;0.0;0.6; 0.5; -0.5; 0.5; -0.5]
turtlebot["bTc_format"] = "xyzqwqxqyqz"

# robotdata = json(turtlebot).data
# resp = JSON.parse(String(take!(IOBuffer(robotdata))))
# tryunpackalltypes!(resp)
# resp

# Actually modify the database
insertrobotdatafirstpose!(cloudGraph, "SESSTURT21", turtlebot)
frd = fetchrobotdatafirstpose(cloudGraph, "SESSTURT21")




# A different robot

hauv = Dict()
hauv["robot"] = "hauv"
hauv["bTc"] = [0.0;0.0;0.0; 1.0; 0.0; 0.0; 0.0]
hauv["bTc_format"] = "xyzqwqxqyqz"
hauv["depth_pointcloud_description"] = "BSONpointcloud"
hauv["depth_color_description"] = ["BSONcolor"]

# robotdata = json(hauv).data
hauv


# Actually modify the databases
insertrobotdatafirstpose!(cloudGraph, "SESSHAUVDEV", hauv)
frd = fetchrobotdatafirstpose(cloudGraph, "SESSHAUVDEV")




















#
