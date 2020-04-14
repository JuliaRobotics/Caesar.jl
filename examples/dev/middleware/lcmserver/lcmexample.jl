
using PyLCM
@pyimport microstrain
@pyimport bot_core

lc = LCM()

function handle_msg(channel, msg_data)
    global lc
    @time msg = microstrain.ins_t[:decode](msg_data)
    @show msg[:utime]

    # msg = microstrain.ins_t()
    nmsg = deepcopy(msg)
    nmsg[:utime] = 12345
    publish(lc, "NEWMSG", nmsg)
end

subscribe(lc, "MICROSTRAIN_IMU", handle_msg)

while true
    handle(lc)
end


function handle_cam(channel, msg_data)
  global lc
  msg = bot_core.image_t[:decode](msg_data)
  @show msg[:width], msg[:height], msg[:pixelformat], msg[:size], typeof(msg[:data]), msg[:data][1]
  ndata = Vector{Int}(msg[:size])
  d = msg[:data]
  for i in 1:5#msg[:size]
    @show convert(Vector{UInt8},d[i]) #ndata[i] = Int(msg[:data][i])
  end
  @show size(ndata)
  publish(lc, "NEWIMAGE", msg)
  nothing
end

subscribe(lc, "CAMLCM_IMAGE", handle_cam)



# for ev in reader
#   ev[:channel] == "MICROSTRAIN_INS" ? println(ev) : nothing
# end

# using PyCall
# pylcm = pyimport("lcm")
# pylcm[:EventLog]("/home/dehann/data/inertialtest/df-backtozero/6point-01.lcm","r")
# for ev in reader
#          ev[:channel] == "MICROSTRAIN_INS" ? println(microstrain.ins_t[:decode](convert(Vector{UInt8}, ev[:data]))) : nothing
#        end


pkgs = [
  "IJulia";
  "Color";
  "Graphs";
  "GraphViz";
  "Gadfly";
  "DataFrames";
  "Distributions";
  "NLsolve";
  "PyCall";
  "PyPlot";
  "HDF5";
  "JLD";
  "JuMP";
  "NLopt";
  "RobotOS"
]

for p in pkgs; Pkg.add(p); end
