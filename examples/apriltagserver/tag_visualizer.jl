using LCMCore
using CaesarLCMTypes
using Rotations, CoordinateTransformations
using MeshCat
using TransformUtils
using AprilTags

vis = Visualizer()
open(vis)
# setobject!(vis["tag01"], Triad(0.2))
# settransform!(vis, Translation(-0.5, -0.5, 0))

function tagPicture(vis::Visualizer,msg::apriltag_t)
    m=homography_to_pose(Array{Float64,2}(msg.homography),300,300,320,240)
    setobject!(vis["tag$(msg.id)"], Triad(0.2))
    settransform!(vis, Translation(m[1:3,4]...) ∘ LinearMap(Quat(m[1:3,1:3])))
    # settransform!(vis, Rotations(m[1:3,1:3]...))
end

function typed_callback(channel::String, msg::apriltag_t)

    @show channel
    @show msg
    tagPicture(vis,msg)
end



lcm=LCM()
subscribe(lcm, "TEST_CHANNEL", typed_callback, apriltag_t)


while true
    handle(lcm)

end
