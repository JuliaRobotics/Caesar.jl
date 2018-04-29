# LCM server

This example shows how to interface with Caesar.jl via LCM: nodes and factors are instantiated through LCM messages. Communication is currently one-directional, __i.e.__, `server.jl` receives messages to grow the factor graph but does not (yet) reply with the latest pose estimates.


Similarly to other libraries, the graph is created by instantiating pose nodes, and constraints on/between those nodes. Nodes are added through the `caesar_pose_node_t.lcm` message type.
It supports the following factor types:

* `Pose3Pose3NH` - a full 6DoF constraint with null hypothesis support (via `caesar_pose_pose_nh_t.lcm`)
* `PartialXYH` - a partial constraint in X, Y and heading, i.e., between local-level/tangent frames at z=0 (via `caesar_pose_pose_xyh_t.lcm`)
* `PartialXYHNH` - same as above, but with support for null hypothesis (via `caesar_pose_pose_xyh_nh_t.lcm`)

There is also limited support for sensor data; at the moment only RGB point clouds are supported, via `caesar_point_cloud_t.lcm`.

## CaesarLCMTypes

Find LCMType definitions at [JuliaRobotics/CaesarLCMTypes.jl repo](https://github.com/JuliaRobotics/CaesarLCMTypes.jl).

## Running the example

### Server

To start the server, run the following command from the example directory:

```sh
julia server.jl

```

### Solver

To start the solver, run the following commands in julia:

```julia
using Caesar
slamindb()
```

The second command will prompt for location and credentials for the two databases.

### Visualization

To start the visualization, run the following commands in julia:

```julia
using Caesar
drawdbdirector()
```

The second command will prompt for location and credentials for the two databases.

## See also

* [caesar-lcm](http://github.com/pvazteixeira/caesar-lcm) - a C++ interface to Caesar.jl over LCM.
