using LCMCore
using CaesarLCMTypes
using Rotations, CoordinateTransformations
using MeshCat
using TransformUtils
using AprilTags

tagData = Dict{Int,apriltag_t}()

function tag_handler(c,m,td)
    td[m.id] = deepcopy(m)
    nothing
end

lcm=LCM()


subscribe(lcm, "TEST_CHANNEL", (m,c)->tag_handler(m,c,tagData), apriltag_t)


@async begin
    while true
        tnow = Int64(time()*1000_000)
        @show keys(tagData)
        for tag in tagData
            if abs(tnow - tag[2].utime) <= 100_000
                msg = tag_landmark_t()
                msg.id = tag[2].id
                msg.bearing = atan2(tag[2].pose[3,2], tag[2].pose[3,3])
                msg.range = norm(tag[2].pose[1:3,4])
                msg.utime = tag[2].utime
                publish(lcm,"Landmarks",encode(msg))
            elseif 100_000 < abs(tnow - tag[2].utime) < 1000_000
                @warn "AprilTag pipeline is taking more than 100ms: $(round(abs(tnow - tag[2].utime)/1000.0, 2))"
            end
            delete!(tagData, tag[1])
        end
        sleep(2)
    end
end

while true
    handle(lcm)
    sleep(0.01)
end







#
