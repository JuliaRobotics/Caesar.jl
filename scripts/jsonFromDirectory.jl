using Revise

using Caesar;
using Caesar.ZmqCaesar;

fg = Caesar.initfg();
config = Dict{String, String}();
zmqConfig = ZmqServer(fg, config, true, "tcp://*:5555");

logdir = "/tmp/Caesar/2019-04-12T18:22:00.152/"

processJsonDir(zmqConfig, logdir, yodamode=true)
