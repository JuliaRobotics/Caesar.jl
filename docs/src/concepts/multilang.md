# Multilanguage Interops

The Caesar framework is not limited to direct Julia use. 

## Static, Shared Object `.so` Compilation
See [Compiling Binaries](@ref compile_binaries).

## ROS Integration
See [ROS Integration](@ref ros_direct).

## Python Direct

For completeness, another design pattern is to wrap Julia packages for direct access from python, see [SciML/diffeqpy](https://github.com/SciML/diffeqpy) as example.

## Caesar SDKs and APIs

The maintainers of Caesar.jl together with NavAbility.io are developing a standardized SDK / API for much easier multi-language / multi-access use of the solver features.  Contact info@navability.io for more information.

!!! note
    2021Q4, Coming Soon!  A new multilanguage SDK is under development and will replace and consolidate the previous methods listed below.

### Previous Generation APIs

The following Github projects provide access to features of Caesar in their language:

* Julia Web interface:
  * [GraffSDK.jl](https://github.com/GearsAD/GraffSDK.jl)

* ZMQ Interface
  * C/C++:
    * [Graff Cpp](https://github.com/MarineRoboticsGroup/graff_cpp)
    * [Caesar LCM](http://github.com/pvazteixeira/caesar-lcm)
    * [Caesar ROS](http://github.com/pvazteixeira/caesar_ros)
  * Python:
    * [GraffSDK.py](https://github.com/nicrip/graff_py) (needs to be updated)
    * [Synchrony_py](https://github.com/nicrip/SynchronySDK_py)

## ZMQ Messaging Interface

Caesar.jl has a ZMQ messaging interface ([interested can see code here](https://github.com/JuliaRobotics/Caesar.jl/blob/master/src/zmq/ZmqCaesar.jl)) that allows users to interact with the solver code base in a variety of ways.  The messaging interface is not meant to replace static `.so` library file compilation but rather provide a more versatile and flexible development strategy.

### Starting the Caesar ZMQ Navigation Server

Start the `Caesar.ZmqCaesar` server in a Julia session with a few process cores and full optimization:

```bash
julia -p4 -O3
```

Then run the following commands, and note these steps have also been [scripted here](https://github.com/JuliaRobotics/Caesar.jl/blob/master/scripts/zmqServer.jl):
```julia
#import the required modules
using Caesar, Caesar.ZmqCaesar

# create empty factor graph and config objects
fg = initfg()
config = Dict{String, String}()
zmqConfig = ZmqServer(fg, config, true, "tcp://*:5555");

# Start the server over ZMQ
start(zmqConfig)

# give the server a minute to start up ...
```

The [current tests are a good place to see some examples](http://github.com/JuliaRobotics/Caesar.jl/tree/master/test/multilangzmq) of the current interfacing functions.  Feel free to change the ZMQ interface for to any of the ZMQ supported modes of data transport, such as [Interprocess Communication (IPC)](http://api.zeromq.org/2-1:zmq-ipc) vs. TCP.


## Alternative Methods

Interfacing from languages like Python may also be achieved using PyCall.jl although little work has been done in the Caesar.jl framework to explore this path.  Julia is itself interactive/dynamic and has plenty of line-by-line and Integrated Development Environment support -- consider trying Julia for your application.
