using Caesar;
using Caesar.ZmqCaesar;
fg = initfg();
config = Dict{String, String}();
zmqConfig = ZmqServer(fg, config, true, "tcp://*:5555");
start(zmqConfig)
