# Local compute version

@show ARGS

#include("parsecommands.jl")  # Hi Kurran, see here


## Load all required packages
using Distributed

"""
Ensure the desired number of julia processes are present.
"""
function check_procs_IIF(cores::Int)
  if cores > 1
    nprocs() < cores ? addprocs(cores-nprocs()) : nothing
  end
end
check_procs_IIF(4) # make sure there are 4 processes waiting before loading packages

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
currdirtime = now()
# currdirtime = "2018-10-28T23:17:30.067"
# currdirtime = "2018-11-03T22:48:51.924"
# currdirtime = "2018-11-07T01:36:52.274"
# currdirtime = "2018-11-08T21:56:16.991"
resultsparentdir = joinpath(datadir, "results")
resultsdir = joinpath(resultsparentdir, "$(currdirtime)")


# When running fresh from new data
include(joinpath(dirname(@__FILE__),"createResultsDirs.jl") )



# Utils required for this processing script
include(joinpath(dirname(@__FILE__),"racecarUtils.jl") )
include(joinpath(dirname(@__FILE__),"cameraUtils.jl") )
include(joinpath(dirname(@__FILE__),"visualizationUtils.jl") )



# @load resultsdir*"/tag_det_per_pose.jld2" tag_bag

tag_bag = detectTagsInImgs(datafolder, imgfolder, resultsdir, camidxs)
# save the tag bag file for future use
@save resultsdir*"/tag_det_per_pose.jld2" tag_bag





fg = main(resultsdir, camidxs, tag_bag)

0






#
