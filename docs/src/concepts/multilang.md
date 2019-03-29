# Multi-Language and Shared Objects

## Multilanguage Interops: Caesar SDKs and APIs
The Caesar framework is not limited to direct Julia use. The following Github projects provide access to features of Caesar in their language:

* Julia Web interface:
  * [GraffSDK.jl](https://github.com/GearsAD/GraffSDK.jl)

* ZMQ Interface
  * C/C++:
    * [Graff Cpp](https://github.com/ MarineRoboticsGroup/graff_cpp)
    * [Caesar LCM](http://github.com/pvazteixeira/  caesar-lcm)
  * Python:
    * [SynchronySDK](https://github.com/nicrip/ SynchronySDK_py)

Contributions are welcome! If you are developing an extension we would like to help, please feel free to contact us (details below).

## ZMQ Messaging Interface

Caesar.jl has a ZMQ messaging interface ([interested can see code here  here](https://github.com/JuliaRobotics/Caesar.jl/blob/master/src/zmq/ZmqCaesar.jl)) that allows users to interact with the solver code base in a variety of ways.  The messaging interface is not meant to replace static `.so` library file compilation--see below---but rather a more versatile and flexible development strategy.

The current known interface implementations to Caesar.jl are:
- C/C++ [GraffCPP](https://github.com/MarineRoboticsGroup/graff_cpp),
- Python [SychronySDK](https://github.com/nicrip/SynchronySDK_py) (needs to be updated),

### Starting the Caesar ZMQ Navigation Server

Start the `Caesar.ZmqCaesar` server in a Julia session with a few process cores:

```bash
julia -p 4
```

Then run the following commands, and note these steps have also been [scripted here](https://github.com/JuliaRobotics/Caesar.jl/blob/master/scripts/zmqServer.jl):
```julia
#import the required modules
using Caesar, Caesar.ZmqCaesar

# create empty factor graph and config objects
fg = Caesar.initfg()
config = Dict{String, String}()
zmqConfig = ZmqServer(fg, config, true, "tcp://*:5555");

# Start the server over ZMQ
start(zmqConfig)

# give the server a minute to start up ...
```

The [current tests are a good place to see some examples](http://github.com/JuliaRobotics/Caesar.jl/tree/master/test/multilangzmq) of the current interfacing functions.

> TODO: expand the ZMQ documentation

### ROS Integration

Yes, but not yet.  See:

> [FAQ: ROS Integration](http://www.juliarobotics.org/Caesar.jl/latest/faq/#ROS-Integration-1)

## Static, Shared Object `.so` Compilation

> [FAQ: Static, Shared Object `.so` Compilation](.http://www.juliarobotics.org/Caesar.jl/latest/faq/#Static,-Shared-Object-.so-Compilation-1)

The future planning for the `Caesar.jl` framework and the mm-iSAM inference algorithm is to use the package: https://github.com/JuliaLang/PackageCompiler.jl so that linkable `.so` files or even full executables can be compiled.  Please add your comments to [this issue discussion](https://github.com/JuliaRobotics/Caesar.jl/issues/210).


## Alternative Methods

Interfacing from languages like Python may also be achieved using PyCall.jl although little work has been done in the Caesar.jl framework to explore this path.  Julia is itself interactive/dynamic and has plenty of line-by-line and Integrated Development Environment support -- consider trying Julia for your application.
