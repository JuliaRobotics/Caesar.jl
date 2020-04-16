# Local compute version

using Pkg
Pkg.activate(@__DIR__)
Pkg.instantiate()
# Pkg.precompile()


## Load all required packages
using Distributed
# addprocs(5) # make sure there are 4 processes waiting before loading packages

@everywhere begin
  using Pkg
  Pkg.activate(@__DIR__)
end

WP = WorkerPool(2:nprocs() |> collect )


# @show ARGS
include("parsecommands.jl")

using DelimitedFiles
using Dates, Statistics
using YAML
using CoordinateTransformations, Rotations, StaticArrays
using ImageCore
using Images, ImageDraw
# using ImageView
using AprilTags
using DataInterpolations

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
include(joinpath(@__DIR__,"configParameters.jl") )


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
  include(joinpath(@__DIR__,"createResultsDirs.jl"))
end


# Utils required for this processing script
include(joinpath(@__DIR__,"racecarUtils.jl") )
include(joinpath(@__DIR__,"cameraUtils.jl") )
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


runnr = parse(Int, parsed_args["folder_name"][end])
joyTimeseries = readdlm(joinpath(datadir,parsed_args["folder_name"],"labRun$(runnr)_joy.csv"), ',')
joyTimeseries = joyTimeseries[2:end,[1,6,8]]
joyTimeseries[:,1] .= joyTimeseries[:,1]*1e-9 # .|> unix2datetime;

# load the detections file to get timestamps
detcData = readdlm(joinpath(datadir,parsed_args["folder_name"],"labRun$(runnr)Detections.csv"), ',')
detcData = detcData[2:end,:]
detcPoseTs = detcData[:,4]*1e-9 .+ detcData[:,3] .|> unix2datetime

## find timeseries segments that go with each pose
joyTsDict = Dict{Symbol, Array{Float64,2}}()
poseTimes = Dict{Symbol, DateTime}(:x0 => detcPoseTs[1])
mask = joyTimeseries[:,1] .< datetime2unix(detcPoseTs[1])
joyTsDict[:x0] = joyTimeseries[mask,:]

for i in 2:length(detcPoseTs)
  mask = datetime2unix(detcPoseTs[i-1]) .<= joyTimeseries[:,1] .< datetime2unix(detcPoseTs[i])
  joyTsDict[Symbol("x$(i-1)")] = joyTimeseries[mask,:]
  poseTimes[Symbol("x$(i-1)")] = detcPoseTs[i-1] # later helper
end

## interpolate up to PyQuest values...

intJoyDict = Dict{Symbol,Vector{Vector{Float64}}}()
# for sym, lclJD = :x1, joyTsDict[:x1]
for (sym, lclJD) in joyTsDict
  if 1 < size(lclJD,1)
    tsLcl = range(lclJD[1,1],lclJD[end,1],length=25)
    intrTrTemp = DataInterpolations.LinearInterpolation(lclJD[:,2],lclJD[:,1])
    intrStTemp = DataInterpolations.LinearInterpolation(lclJD[:,3],lclJD[:,1])
    newVec = Vector{Vector{Float64}}()
    for tsL in tsLcl
      newVal = zeros(4)
      newVal[1] = intrTrTemp(tsL)
      newVal[2] = intrStTemp(tsL)
      push!(newVec, newVal)
    end
    # currently have no velocity values
    intJoyDict[sym] = newVec
  else
    intJoyDict[sym] = [zeros(4) for i in 1:25]
  end
end

## More code required

# add the NeuralPose2Pose2 factor in Main workspace
include( joinpath(dirname(pathof(Caesar)), "..", "examples", "learning", "hybrid", "NeuralPose2Pose2", "FluxPose2Pose2.jl") )

include(joinpath(@__DIR__, "LoadPyNNText.jl"))

## load the required models into common predictor
# modelX =  loadTfModelIntoFlux(dest...)



## run the solution


fg = main(WP, resultsdir, camidxs, tag_bag, jldfile=parsed_args["jldfile"], failsafe=parsed_args["failsafe"], show=parsed_args["show"], odopredfnc=PyTFOdoPredictorPoint2, joyvel=intJoyDict, poseTimes=poseTimes, multiproc=false  )


## development

#
# 0
#
# fg = initfg()
#
# addVariable!(fg, :x0, Pose2, timestamp=poseTimes[:x0])
# addVariable!(fg, :x1, Pose2, timestamp=poseTimes[:x1])
#
# pp2 = PriorPose2(MvNormal(zeros(3),LinearAlgebra.diagm([0.01;0.01;0.005].^2)))
# addFactor!(fg, [:x0], pp2)
#
# # the neural factor
# DXmvn = MvNormal(zeros(3),LinearAlgebra.diagm([0.01;0.01;0.005].^2))
# odopredfnc=PyTFOdoPredictorPoint2
# joyvel=intJoyDict
# pp = PyNeuralPose2Pose2(odopredfnc,joyvel[:x0],DXmvn,0.4)
# addFactor!(fg, [:x0;:x1], pp)





# fails
# key 1 not found
# julia101 -p 4 apriltag_and_zed_slam.jl --previous "2018-11-09T01:42:33.279" --jldfile "racecar_fg_x299.jld2" --folder_name "labrun7" --failsafe


#
