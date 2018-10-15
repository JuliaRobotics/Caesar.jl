using Revise
using JSON, ZMQ
using Base.Test
using Caesar
using Unmarshal

addOdo2DJson = "{\n  \"covariance\": [\n    [\n      0.1,\n      0.0,\n      0.1\n    ],\n    [\n      0.1,\n      0.0,\n      0.1\n    ],\n    [\n      0.1,\n      0.0,\n      0.1\n    ]\n  ],\n  \"measurement\": [\n    10.0,\n    0.0,\n    1.0471975511965976\n  ],\n  \"robot_id\": \"Hexagonal\",\n  \"session_id\": \"cjz002\",\n  \"type\": \"addOdometry2D\"\n}";
addLandmark2DJson = "{\n  \"landmark_id\": \"l1\",\n  \"robot_id\": \"Hexagonal\",\n  \"session_id\": \"cjz002\",\n  \"type\": \"addLandmark2D\"\n}";
addOdo2DCmd = JSON.parse(addOdo2DJson)
addLandmark2DCmd = JSON.parse(addLandmark2DJson)

fg = Caesar.initfg()
config = Dict{String, String}()

# Send a command locally.
function sendCmd(config, fg, cmd::Dict{String, Any})::String
    callback = getfield(Caesar, Symbol(cmd["type"]))
    resp = callback(config, fg, cmd)
    return JSON.json(resp)
end

# Add two variables.
for x in ["x0", "x1", "x2"]
    addVariableCmd = Dict{String, Any}("type" => "addVariable", "variable" => JSON.parse(JSON.json(VariableRequest(x, "Pose2", 100, ["TEST"]))))
    @test sendCmd(config, fg, addVariableCmd) == "{\"status\":\"OK\",\"id\":\"$x\"}"
end
# ls to see that they are added to the graph.
lsCmd = Dict{String, Any}("type" => "ls", "filter" => JSON.parse(JSON.json(lsRequest("true", "true"))))
@test sendCmd(config, fg, lsCmd) == "{\"variables\":[{\"factors\":[],\"id\":\"x0\"},{\"factors\":[],\"id\":\"x1\"},{\"factors\":[],\"id\":\"x2\"}]}"

# Add a prior factor
dist = Dict{String, Any}("type" => "Normal", "mean" => [0.0 0.0 0.0], "cov" => zeros(3,3))
factCmd = Dict{String, Any}("type" => "addFactor", "factor" => JSON.parse(JSON.json(FactorRequest("Prior", ["x0", "x1"], [dist]))))
sendCmd(config, fg, factCmd)

# Testing
factorRequest = Caesar.FactorRequest("Prior", String["x0", "x1"], Dict{String,Any}[Dict{String,Any}(Pair{String,Any}("type", "Normal"),Pair{String,Any}("mean", Any[Any[0.0], Any[0.0], Any[0.0]]),Pair{String,Any}("cov", Any[Any[0.0, 0.0, 0.0], Any[0.0, 0.0, 0.0], Any[0.0, 0.0, 0.0]]))])

#######
### STATE OF THE ART. BEYOND HERE THERE BE KRAKENS (AND EXCEPTIONS, YAAARGH!)
#######

# Get a node to make sure it's all good.
# getNodeCmd = Dict{String, Any}("type" => "ls", "id" => "x0")

# Call batch solve.
# batchSolveCmd = Dict{String, Any}("type" => "batchSolve")
# result = sendCmd(config, fg, lsCmd)
# @test result["status"] == "OK"
