using Revise
using JSON, ZMQ
using Base.Test
using Caesar
using Unmarshal

addOdo2DJson = "{\n  \"covariance\": [\n    [\n      0.1,\n      0.0,\n      0.1\n    ],\n    [\n      0.1,\n      0.0,\n      0.1\n    ],\n    [\n      0.1,\n      0.0,\n      0.1\n    ]\n  ],\n  \"measurement\": [\n    10.0,\n    0.0,\n    1.0471975511965976\n  ],\n  \"robot_id\": \"Hexagonal\",\n  \"session_id\": \"cjz002\",\n  \"type\": \"addOdometry2D\"\n}";
addLandmark2DJson = "{\n  \"landmark_id\": \"l1\",\n  \"robot_id\": \"Hexagonal\",\n  \"session_id\": \"cjz002\",\n  \"type\": \"addLandmark2D\"\n}";
addOdo2DCmd = JSON.parse(addOdo2DJson)
addLandmark2DCmd = JSON.parse(addLandmark2DJson)
shutdownCmd = Dict{String, Any}("type" => "shutdown")
emptyCmd = Dict{String, Any}()
unknownCmd = Dict{String, Any}("type" => "NOPENOPENOPE")
mockServerCmd = Dict{String, Any}("type" => "toggleMockServer", "isMockServer" => "true")
addVariableCmd = Dict{String, Any}("type" => "addVariable", "variable" => JSON.parse(JSON.json(VariableRequest("x0", "Pose2", 100, ["TEST"]))))

## ....
# addOdometry2D()
##

function sendCmd(cmd::Dict{String, Any})::String
    ctx=Context()
    sock=Socket(ctx, REQ)
    ZMQ.connect(sock, "tcp://localhost:5555")
    ZMQ.send(sock, JSON.json(cmd))
    msg = ZMQ.recv(sock)
    out=convert(IOStream, msg)
    str = String(take!(out))
    ZMQ.close(sock)
    return str
end

# Send various failure commands
@test sendCmd(unknownCmd) == "{\"status\":\"ERROR\",\"error\":\"Command 'NOPENOPENOPE' not a valid command.\"}"

@test sendCmd(emptyCmd) == "{\"status\":\"ERROR\",\"error\":\"Command 'ERROR_NOCOMMANDPROVIDED' not a valid command.\"}"

# All our tests
@test sendCmd(addVariableCmd) == "{\"label\":\"x0\",\"status\":\"OK\"}"

# Lastly mock server test
@test sendCmd(mockServerCmd) == "{\"status\":\"OK\",\"isMockServer\":\"true\"}"
@test sendCmd(addOdo2DCmd) == "{\"status\":\"OK\", \"label\": \"x0\"}"
# Disable it
mockServerCmd["isMockServer"]="false"
@test sendCmd(mockServerCmd) == "{\"status\":\"OK\",\"isMockServer\":\"false\"}"


# Send the shutdown command - cannot be mocked.
@test sendCmd(shutdownCmd) == "{\"status\":\"OK\"}"
