using LCMCore
using CaesarLCMTypes

function typed_callback(channel::String, msg::apriltag_t)
    @show channel
    @show msg
end
lcm=LCM()
subscribe(lcm, "TEST_CHANNEL", typed_callback, apriltag_t)


while true
    handle(lcm)
end
