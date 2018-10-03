# using LCMCore
# using CaesarLCMTypes
using Rotations, CoordinateTransformations
using MeshCat
using TransformUtils
using AprilTags

include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","cameraUtils.jl"))

vis = Visualizer()
open(vis)

setobject!(vis["home"], Triad(1.0))

detector = AprilTagDetector()
# setobject!(vis["tag01"], Triad(0.2))
# settransform!(vis, Translation(-0.5, -0.5, 0))

fx = fy = 80.0
cx,cy = (320,240)
k1,k2 = (0.0,0.0)
camK = [fx 0 cx; 0 fy cy; 0 0 1]

tagsize = 0.172


function tagPicture(vis::Visualizer,msg::apriltag_t, drawdict)
    # m=homography_to_pose(Array{Float64,2}(msg.homography),300,300,320,240)
    m = msg.pose
    tkey = "tag$(msg.id)"
    if !haskey(drawdict, tkey)
        setobject!(vis[tkey], Triad(0.2))
        drawdict[tkey] = 1
    end
    # wTc=LinearMap(RotX(pi/2))
    # cTt=Translation(m[1:3,4]...) ∘ LinearMap(Quat(m[1:3,1:3]))
    # settransform!(vis[tkey], wTc ∘ cTt)
    P = Vector{Vector{Float64}}()
    for i in 1:4
      push!(P, msg.P[i,:])
    end
    tag = AprilTag(msg.familyName, msg.id, msg.hammingDistance, Float32(0.0), P)
    T,Q, = getAprilTagTransform(tag,
                                camK,
                                k1,
                                k2,
                                tagsize )
    cTt = T ∘ LinearMap(Q)
    settransform!(vis[tkey], cTt)
    nothing
end

function typed_callback(channel::String, msg::apriltag_t, drawdict)
    tagPicture(vis,msg, drawdict)
end

drawdict = Dict{String, Int}()

lcm=LCM()
subscribe(lcm, "TEST_CHANNEL", (m,c)->typed_callback(m,c,drawdict), apriltag_t)



while true
  handle(lcm)
end
