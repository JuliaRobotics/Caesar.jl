

# Link with ROSbag infrastructure via rospy
using Pkg
ENV["PYTHON"] = "/usr/bin/python3"
Pkg.build("PyCall")

using PyCall
using OpenSSL # workaround attempt, https://github.com/JuliaWeb/OpenSSL.jl/issues/9
using RobotOS

include("ROSJSONtoJSON.jl")


##

# Submodule for converting PointClouds
using Caesar


wPC  = Caesar._PCL.PointCloud()
wPC2 = Caesar._PCL.PCLPointCloud2(wPC)
rmsg = Caesar._PCL.toROSPointCloud2(wPC2);

@info "In-situ test complete of tricky ROS and _PCL loading."

##

using Colors
using ImageMagick, FileIO
using Images

# likely packages needed to ingest various different sensor data types and serialize them for NVA
using Dates, TimeZones
using Random
using JSON
using BSON, Serialization
using FixedPointNumbers
using StaticArrays
using DocStringExtensions
# using DataStructures

##

# http://wiki.ros.org/laser_geometry
lg = pyimport("laser_geometry.laser_geometry")

include(joinpath(@__DIR__, "SystemState.jl"))

# include(joinpath(dirname(dirname(@__DIR__)), "NonDuplicatingUtils.jl")) # legacy

# actual handlers
include(joinpath(@__DIR__, "IngestLidar3D.jl"))
include(joinpath(@__DIR__, "IngestRadar2D.jl"))
include(joinpath(@__DIR__, "IngestLaserScan2D.jl"))
include(joinpath(@__DIR__, "IngestIMU.jl"))
include(joinpath(@__DIR__, "IngestCameraRaw.jl"))
include(joinpath(@__DIR__, "IngestCameraInfo.jl"))
include(joinpath(@__DIR__, "IngestTFMessage.jl"))
include(joinpath(@__DIR__, "IngestOdometry.jl"))
include(joinpath(@__DIR__, "IngestGPS.jl"))
include(joinpath(@__DIR__, "IngestTwist.jl"))



## Automation code that converts user options on which topics and types to subscribe to in a ROSbag.

function SubscribeROSTopicOptions(;kw...)
  di = Dict{String,Any}()
  for (k,v) in kw
      di[string(k)] = v
  end
  di
end


Base.@kwdef struct SubscribeROSTopicInput
  topicname::String
  msgtype
  callback
end


function addROSSubscriber!(bagSub::RosbagSubscriber, inp::SubscribeROSTopicInput, subsargs::Tuple=() )
  @show inp.msgtype
  bagSub(inp.topicname, inp.msgtype, inp.callback, subsargs )
end

"""
    _suppressPushExisting!

Piece of defensive coding, if graph upload from ROS with BashingMapper breaks during process, this helps resume the "upload"
"""
function _suppressPushExisting!(
  nfg::AbstractDFG,
  state::SystemState,
  vars::AbstractVector{Symbol},
  userPushNva::Bool,
  userPushBlobs::Bool
)
  # skip if not allow suppress
  !state.suppressPush && (return false)
  lbl = Symbol("x$(state.var_index)")
  if lbl in vars
    state.pushNva = false
    # not a perfect check, assumes all or nothing is already up there
    state.pushBlobs = if 0 === length(listBlobEntries(nfg, lbl))
      @warn "might upload blobs for $lbl"
      userPushBlobs
      return false
    else
      @warn "skipping uploads of $lbl"
      false
      return true
    end
  else
    # regular case of nothing up on nfg yet so just add as stuff as normal
    state.pushNva = userPushNva
    state.pushBlobs = userPushBlobs
    return false
  end
  #
  error("should get here")
end


## a common main loop

function main(
  nfg::AbstractDFG, 
  bagfile, 
  userWantsSubsOnROS; 
  iters::Integer=50, 
  stripeKeyframe=5, 
  pushNva=true,
  pushBlobs=false, 
  cur_variable = "x0", # should already be uploaded in a previous step
  var_index = 1        # this is the next variable to add -- i.e. "x1"
)
  #

  @info "Hit CTRL+C to exit and save the graph..."
  
  if pushNva
    x0 = NvaSDK.getVariable(client, context, cur_variable) |> fetch
    @assert !(x0 === nothing) "The indicated starting variable $cur_variable was not found"
  end

  # subscriber for the bagfile
  bagSubscriber = RosbagSubscriber(bagfile)

  # System state
  systemstate = SystemState(;
                    stripeKeyframe,  # TODO remove, use handler option instead
                    pushBlobs, 
                    pushNva,
                    cur_variable,
                    var_index
                )

  # all the subscribers the user requested
  for (usrInp, usrOpt) in userWantsSubsOnROS
    @info "Adding ROS subscriber " context usrInp usrOpt
    addROSSubscriber!(bagSubscriber, usrInp, (client, context, systemstate, usrOpt))
  end

  vars = listVariables(nfg)

  @info "subscribers have been set up; entering main loop"
  # loop_rate = Rate(20.0)
  while loop!(bagSubscriber)
    # defensive suppress state.pushNva and .pushBlobs if variable already there -- i.e. trying to resume an upload
    _suppressPushExisting!(nfg, systemstate, vars, pushNva, pushBlobs)
    # regular processing steps
    iters -= 1
    iters < 0 ? break : nothing
  end
  
  @info "Exiting"
  systemstate
end


##