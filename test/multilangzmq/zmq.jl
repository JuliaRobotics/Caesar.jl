using Revise
using JSON, ZMQ
# using Caesar
using Unmarshal

addOdo2DJson = "{\n  \"covariance\": [\n    [\n      0.1,\n      0.0,\n      0.1\n    ],\n    [\n      0.1,\n      0.0,\n      0.1\n    ],\n    [\n      0.1,\n      0.0,\n      0.1\n    ]\n  ],\n  \"measurement\": [\n    10.0,\n    0.0,\n    1.0471975511965976\n  ],\n  \"robot_id\": \"Hexagonal\",\n  \"session_id\": \"cjz002\",\n  \"type\": \"AddOdometry2D\"\n}";
addLandmark2DJson = "{\n  \"landmark_id\": \"l1\",\n  \"robot_id\": \"Hexagonal\",\n  \"session_id\": \"cjz002\",\n  \"type\": \"addLandmark2D\"\n}";
addOdoPayload = JSON.parse(addOdo2DJson)
addLandmark2DPayload = JSON.parse(addLandmark2DJson)
shutdownCmd = Dict{String, Any}("type" => "shutdown")
emptyCmd = Dict{String, Any}()
unknownCmd = Dict{String, Any}("type" => "NOPENOPENOPE")

## ....
addOdometry2D()
##

# Set up a connection
ctx=Context()
sock=Socket(ctx, REQ)
ZMQ.connect(sock, "tcp://localhost:5555")

# Send various commands
ZMQ.send(sock, JSON.json(emptyCmd))
ZMQ.send(sock, JSON.json(unknownCmd))


# Send the shutdown command
ZMQ.send(sock, JSON.json(shutdownCmd))
