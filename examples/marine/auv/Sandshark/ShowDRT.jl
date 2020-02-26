
## Load necessary code

using Pkg
Pkg.activate(@__DIR__)
pkg"instantiate"
pkg"precompile"


using FileIO


include(joinpath(@__DIR__, "CommonUtils.jl"))
# include(joinpath(@__DIR__, "Plotting.jl"))


## load specific case


pargs = if !isdefined(Main, :parsed_args)
  parse_commandline()
else
  parsed_args
end

##

pargs["reportDir"] = if !haskey(pargs, "reportDir") && isdefined(Main, :fg)
  getLogPath(fg)
else
  # "/tmp/caesar/2020-02-23T01:43:32.222/fg_before_x291.tar.gz"
  "/tmp/caesar/2020-02-23T01:43:32.222/fg_after_x1391.tar.gz"
2020-02-24T04:28:41.955
  # pargs["reportDir"]
end


# @show pargs["reportDir"]
# @show splitpath(pargs["reportDir"])

# load the factor graph needed
fg = if true || !isdefined(Main, :fg)
  println("going to load fg")
  fg = LightDFG{SolverParams}(params=SolverParams())
  @show pathElem = splitpath(pargs["reportDir"])
  @show getSolverParams(fg).logpath = joinpath(pathElem[1:end-1]...)
  loadDFG(pargs["reportDir"], Main, fg)
  fg
else
  fg
end


# random temp data
drt_data, TTm, XXm, YYm, XXf, YYf = loadResultsDRT(fg)


# load here due to name conflict with Point2
using MakieLayout
using Makie

include(joinpath(@__DIR__, "MakiePlotsFG.jl"))


## get a portion of the graph

fg2 = deepcopy(fg)

# trim down the factor graph
klist = (ls(fg, r"x\d") |> sortDFG)[end-120:end]
rmlist = setdiff(ls(fg), klist)
rmfct = union((x->ls(fg, x)).(rmlist)...)
(x->deleteFactor!(fg, x)).(rmfct)
(x->deleteVariable!(fg, x)).(rmlist)

##

DRT = true
PPE = true
REF = false

include(joinpath(@__DIR__, "MakiePlotImgNoDock.jl"))


scene
##


@show scenepath = joinLogPath(fg, "DRTafter.png")
Base.rm(scenepath, force=true)
Makie.save(scenepath, scene)



##
