
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


pargs["reportDir"] = if !haskey(pargs, "reportDir") && isdefined(Main, :fg)
  getLogPath(fg)
else
  # "/tmp/caesar/2020-02-03T03:07:45.938/fg_after_x1181.tar.gz"
  "/tmp/caesar/2020-02-22T02:21:20.777/fg_after_x781.tar.gz"
end



# @show pargs["reportDir"]
# @show splitpath(pargs["reportDir"])

# load the factor graph needed
fg = if !isdefined(Main, :fg)
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


##

include(joinpath(@__DIR__, "MakiePlotImg.jl"))



##


Makie.save("/tmp/test.png", scene)



##

using Glob


pargs["plotSeriesBeliefs"] = 0

# plot a series of frames
if 0 <= pargs["plotSeriesBeliefs"]

global frame = 0 # still finiky, leave at 0
pattern = "fg_after_x"
ext = ".tar.gz"
files = glob("$(pattern)*$ext", getLogPath(fg))
indiv = splitpath.(files) .|> x->x[end]
presort = (x->x[length(pattern)+1:end-length(ext)]).(indiv)
sorted = parse.(Int, presort) |> sortperm
# reduce the number of frames, if requested
indiv = pargs["plotSeriesBeliefs"] == 0 ? indiv[sorted] : indiv[sorted[(frame+1):pargs["plotSeriesBeliefs"]]]
# get range from last file in glob
fg = LightDFG{SolverParams}(params=SolverParams(logpath=getLogPath(fg)))
loadDFG(joinLogPath(fg, indiv[end]), Main, fg)
# drt_data, TTm, XXm, YYm, XXf, YYf = loadResultsDRT(fg)
minmax = getRangeCartesian(fg,r"x\d",digits=-1)
# xmin=minmax[1,1]-10;xmax=minmax[1,2]+10;ymin=minmax[2,1]-10;ymax=minmax[2,2]+10;
Base.mkpath(joinLogPath(fg, "background"))
for ind in indiv
  global fg
  global frame += 1
  println("frame $frame of $(length(indiv))")
  fname = split(ind, '.')[1]
  if pargs["skip"] && isfile(joinLogPath(fg,"background/$fname.png"))
    @info "skip $fname since its already there"
    continue
  end
  fg = LightDFG{SolverParams}(params=SolverParams(logpath=getLogPath(fg)))
  loadDFG(joinLogPath(fg, ind), Main, fg)
  nvars = length(ls(fg, r"x\d"))
  # if nvars < 15
  #   continue
  # end
  try

    include(joinpath(@__DIR__, "MakiePlotImg.jl"))

    Base.rm(joinLogPath(fg,"background/$fname.png"), force=true)
    Makie.save(joinLogPath(fg,"background/$fname.png"), scene)
  catch ex
    @error ex
  end
  GC.gc()
end

end
