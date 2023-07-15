#using Revise

using Caesar, ZMQ
# using JSON
using Test

##

@testset "test CaesarZMQExt" begin
##

addOdo2DPayload = "{\n  \"covariance\": [\n    [\n      0.1,\n      0.0,\n      0.1\n    ],\n    [\n      0.1,\n      0.0,\n      0.1\n    ],\n    [\n      0.1,\n      0.0,\n      0.1\n    ]\n  ],\n  \"measurement\": [\n    10.0,\n    0.0,\n    1.0471975511965976\n  ],\n  \"robot_id\": \"Hexagonal\",\n  \"session_id\": \"cjz002\",\n  \"type\": \"AddOdometry2D\"\n}"
addLandmark2DPayload = "{\n  \"landmark_id\": \"l1\",\n  \"robot_id\": \"Hexagonal\",\n  \"session_id\": \"cjz002\",\n  \"type\": \"addLandmark2D\"\n}"

# addOdometry2D()
@error "CaesarZMQExt serverTest currently suppressed, must be updated."


##
end
