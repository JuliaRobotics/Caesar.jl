
export RosbagWriter

# assumed this path was added as part of the bag subscriber code
# pushfirst!(PyVector(pyimport("sys")."path"), @__DIR__ )

writeRosbagPy = pyimport("rosbagWriter")
RosbagWriter = writeRosbagPy."RosbagWriter"


# Base.@kwdef struct RosbagWriter
#   bagfile::String
#   bag::PyObject = writeRosbag(bagfile)
# end

# function write(rbw::RosbagWriter, channel::AbstractString, msg)
#   rbw.bag.write_message(channel, msg)
# end

# function close()

