# Local compute version

using Pkg
Pkg.activate(@__DIR__)
Pkg.instantiate()

# load the necessary Python bindings
include(joinpath(@__DIR__,"PythonTensorFlowUsage.jl"))

Pkg.precompile()


## Load all required packages
using Distributed
addprocs(5) # make sure there are 4 processes waiting before loading packages

@everywhere begin
  using Pkg
  Pkg.activate(@__DIR__)
end

WP = WorkerPool(2:nprocs() |> collect )


# @show ARGS
include("parsecommands.jl")


using Dates, Statistics
using CoordinateTransformations, Rotations, StaticArrays
using ImageCore
using Images, ImageDraw
# using ImageView
using AprilTags

using RoME
using Caesar
0
@everywhere using Caesar
@everywhere using JLD2


@everywhere begin
# using Fontconfig
# using Compose
using Gadfly
using RoMEPlotting
using Cairo
# using FileIO
# using GeometryTypes # using MeshCat
end

# setup configuration
using YAML

include(joinpath(dirname(@__FILE__),"configParameters.jl") )


# Figure export folder
global currdirtime = now()
if parsed_args["previous"] != ""
  # currdirtime = "2018-11-07T01:36:52.274"
  currdirtime = parsed_args["previous"]
end

resultsparentdir = joinpath(datadir, "results")
resultsdir = joinpath(resultsparentdir, "$(currdirtime)")

if parsed_args["previous"] == ""
  # When running fresh from new data
  include(joinpath(dirname(@__FILE__),"createResultsDirs.jl"))
end


# Utils required for this processing script
include(joinpath(dirname(@__FILE__),"racecarUtils.jl") )
include(joinpath(dirname(@__FILE__),"cameraUtils.jl") )
# include(joinpath(dirname(@__FILE__),"visualizationUtils.jl") )



@everywhere function plotRacecarInterm(fgl::G, resultsdirl, psyml::Symbol)::Nothing where G <: AbstractDFG
  @show ls(fgl)
  pl = drawPosesLandms(fgl, spscale=0.1, drawhist=false, meanmax=:mean) #,xmin=-3,xmax=6,ymin=-5,ymax=2);
  Gadfly.draw(PNG(joinpath(resultsdirl, "images", "$(psyml).png"),15cm, 10cm),pl)
  pl = drawPosesLandms(fgl, spscale=0.1, meanmax=:mean) # ,xmin=-3,xmax=3,ymin=-2,ymax=2);
  Gadfly.draw(PNG(joinpath(resultsdirl, "images", "hist_$(psyml).png"),15cm, 10cm),pl)
  # pl = plotPose2Vels(fgl, Symbol("$(psyml)"), coord=Coord.Cartesian(xmin=-1.0, xmax=1.0))
  # Gadfly.draw(PNG(joinpath(resultsdirl, "images", "vels_$(psyml).png"),15cm, 10cm),pl)
  # save combined image with tags
  nothing
end



global tag_bag = Dict()
if parsed_args["previous"] == ""
  tag_bag = detectTagsInImgs(datafolder, imgfolder, resultsdir, camidxs, iterposes=parsed_args["iterposes"])
  # save the tag bag file for future use
  @save resultsdir*"/tag_det_per_pose.jld2" tag_bag
else
  @load resultsdir*"/tag_det_per_pose.jld2" tag_bag
end

## Load joystick time series data




fg = main(WP, resultsdir, camidxs, tag_bag, jldfile=parsed_args["jldfile"], failsafe=parsed_args["failsafe"], show=parsed_args["show"], odopredfnc=PyTFOdoPredictorPoint2, joysticktimeseries=joyTImeseries  )



0


# fails
# key 1 not found
# julia101 -p 4 apriltag_and_zed_slam.jl --previous "2018-11-09T01:42:33.279" --jldfile "racecar_fg_x299.jld2" --folder_name "labrun7" --failsafe



#
