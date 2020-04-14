using LCMCore
using CaesarLCMTypes
using Rotations, CoordinateTransformations
using MeshCat
using TransformUtils
using AprilTags

vis = Visualizer()
open(vis)

setobject!(vis["home"], Triad(1.0))

detector = AprilTagDetector()
# setobject!(vis["tag01"], Triad(0.2))
# settransform!(vis, Translation(-0.5, -0.5, 0))

function tagPicture(vis::Visualizer,msg::apriltag_t, drawdict)
    # m=homography_to_pose(Array{Float64,2}(msg.homography),300,300,320,240)
    m = msg.pose
    tkey = "tag$(msg.id)"
    if !haskey(drawdict, tkey)
        setobject!(vis[tkey], Triad(0.2))
        drawdict[tkey] = 1
    end
    wTc=LinearMap(RotX(pi/2))
    cTt=Translation(m[1:3,4]...) ∘ LinearMap(Quat(m[1:3,1:3]))
    settransform!(vis[tkey], wTc ∘ cTt)
    #settransform!(vis[tkey], cTt)
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
