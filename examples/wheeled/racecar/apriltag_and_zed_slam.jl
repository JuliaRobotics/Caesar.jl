# Local compute version

# @show ARGS
include("parsecommands.jl")


## Load all required packages
using Distributed

# check_procs(4) # make sure there are 4 processes waiting before loading packages

using Dates, Statistics
@everywhere using Caesar
@everywhere using JLD2
using CoordinateTransformations, Rotations, StaticArrays

using AprilTags
using Images, ImageView, ImageDraw

@everywhere begin
using Fontconfig
using Cairo
using Compose
using Gadfly
using RoMEPlotting
# using FileIO
# using GeometryTypes # using MeshCat
end

# setup configuration
using YAML

include(joinpath(dirname(@__FILE__),"configParameters.jl") )


# Figure export folder
global currdirtime = now()
if parsed_args["previous"] != ""
    # currdirtime = "2018-10-28T23:17:30.067"
    # currdirtime = "2018-11-03T22:48:51.924"
    # currdirtime = "2018-11-07T01:36:52.274"
    # currdirtime = "2018-11-08T21:56:16.991"
  # currdirtime = "2018-10-28T23:17:30.067"
  # currdirtime = "2018-11-03T22:48:51.924"
  # currdirtime = "2018-11-07T01:36:52.274"
  currdirtime = parsed_args["previous"]
end

resultsparentdir = joinpath(datadir, "results")
resultsdir = joinpath(resultsparentdir, "$(currdirtime)")

if parsed_args["previous"] == ""
  # When running fresh from new data
  include("createResultsDir.jl")
end


# Utils required for this processing script
include(joinpath(dirname(@__FILE__),"racecarUtils.jl") )
include(joinpath(dirname(@__FILE__),"cameraUtils.jl") )
include(joinpath(dirname(@__FILE__),"visualizationUtils.jl") )



global tag_bag = Dict()
if parsed_args["previous"] != ""
  tag_bag = detectTagsInImgs(datafolder, imgfolder, resultsdir, camidxs)
  # save the tag bag file for future use
  @save resultsdir*"/tag_det_per_pose.jld2" tag_bag
else
  @load resultsdir*"/tag_det_per_pose.jld2" tag_bag
end




fg = main(resultsdir, camidxs, tag_bag, jldfile=parsed_args["jldfile"])

0






#
