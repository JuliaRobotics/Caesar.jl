using Caesar, Caesar.ZmqCaesar
using JSON, ZMQ
using Unmarshal
using Test

# @testset "ZMQ End-To-End Test" begin

emptyCmd = Dict{String, Any}()
unknownCmd = Dict{String, Any}("request" => "NOPENOPENOPE")
mockServerCmd = Dict{String, Any}("request" => "toggleMockServer", "payload" => Dict{String, Any}("isMockServer" => "true"))
addVariableCmd = Dict{String, Any}("request" => "addVariable", "payload" => JSON.parse(JSON.json(VariableRequest("x0", "Pose2", 100, ["TEST"]))))
lsCmd = Dict{String, Any}("request" => "ls", "payload" => Dict{String, Any}("variables" => "true", "factors" => "true"))
shutdownCmd = Dict{String, Any}("request" => "shutdown")
addOdo2DCmd = Dict{String, Any}(
    "request" => "addOdometry2D",
    "payload" => JSON.parse("{\n  \"covariance\": [\n    [\n      0.1,\n      0.0,\n      0.1\n    ],\n    [\n      0.1,\n      0.0,\n      0.1\n    ],\n    [\n      0.1,\n      0.0,\n      0.1\n    ]\n  ],\n  \"measurement\": [\n    10.0,\n    0.0,\n    1.0471975511965976\n  ],\n  \"robot_id\": \"Hexagonal\",\n  \"session_id\": \"cjz002\"}"));

@info "Starting the ZMQ server in an @async..."
@async begin
    fg = Caesar.initfg()
    config = Dict{String, String}()
    zmqConfig = ZmqServer(fg, config, true, "tcp://*:5555")
    start(zmqConfig)
end
sleep(20)
@info "Started the ZMQ server!"


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

# Sinmple operational tests
@test sendCmd(addVariableCmd) == "{\"status\":\"OK\",\"id\":\"x0\"}"
@test sendCmd(lsCmd) == "{\"variables\":[{\"factors\":[],\"id\":\"x0\"}],\"status\":\"OK\"}"

# Lastly mock server test
@test sendCmd(mockServerCmd) == "{\"status\":\"OK\",\"isMockServer\":\"true\"}"
@test sendCmd(addOdo2DCmd) == "{\"status\":\"OK\"}"
# Disable it
mockServerCmd["payload"]["isMockServer"]="false"
@test sendCmd(mockServerCmd) == "{\"status\":\"OK\",\"isMockServer\":\"false\"}"

# Send the shutdown command - cannot be mocked.
@test sendCmd(shutdownCmd) == "{\"status\":\"OK\"}"
# end
