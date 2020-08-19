




## Get user commands
# Populates `parsed_args`
include(joinpath(dirname(@__DIR__),"parsecommands.jl"))

# Assume `fec` will be defined by script generating the data, else load --previous
if length(ARGS) == 0 && !@isdefined(fec)
  @info "loading" parsed_args["previous"]
  parsed_args["folder_name"] = "labrun2"
  parsed_args["remoteprocs"] = 0
  parsed_args["localprocs"] = 1
  parsed_args["batch_resolve"] = false
  parsed_args["vis2d"] = true
  parsed_args["vis3d"] = false
  parsed_args["imshow"] = true
  parsed_args["msgloops"] = 3000
  parsed_args["usesimmodels"] = true
end

# bring up the multiprocess cluster
!@isdefined(fec) && include(joinpath(@__DIR__, "CarFeedCommonSetup.jl"))


dfg = if @isdefined(fec)
  fec.slam.dfg
else
  dfg = initfg()
  getSolverParams(dfg).logpath=parsed_args["previous"]
  loadDFG!(dfg, joinLogPath(dfg, "finalGraph"))
  dfg
end




## =========================================================================================
# interpose results
## =========================================================================================


allD = jsonResultsSLAM2D(dfg)

allStr = JSON2.write(allD)

fid = open(joinLogPath(dfg, "$(runfile)_results_prebatch.json"),"w")
println(fid, allStr)
close(fid)

## save the factor graph

if parsed_args["savedfg"]
  saveDFG(dfg, joinLogPath(dfg, "fg_final_prebatch"))
end


## draw results

if parsed_args["vis2d"]


pl = plotSLAM2D(dfg, drawhist=false)
pl |> PDF(joinLogPath(dfg,"fg_final_prebatch.pdf"))

if parsed_args["report_factors"]
  reportFactors(dfg, Pose2Pose2, show=false)
  reportFactors(dfg, Pose2Pose2, lsf(dfg, r"l"),prefix="LM_", show=false)
end

end

## batch resolve

if parsed_args["batch_resolve"]

# fg2 = deepcopy(dfg)

enableSolveAllNotDRT!(dfg)
# dontMarginalizeVariablesAll!(dfg)
# foreach(x->setSolvable!(dfg, x, 1), ls(dfg))
# foreach(x->setSolvable!(dfg, x, 1), lsf(dfg))

tree, smt, hist = solveTree!(dfg, storeOld=true)

if parsed_args["savedfg"]
  saveDFG(dfg, joinLogPath(dfg, "fg_final_resolve"))
end

# report a second time after the resolve
if parsed_args["report_factors"]
  reportFactors(dfg, Pose2Pose2, show=false)
  reportFactors(dfg, Pose2Pose2, lsf(dfg, r"l"),prefix="LM_", show=false)
end

end

## after resolve interpose results

allD = jsonResultsSLAM2D(dfg)

allStr = JSON2.write(allD)

fid = open(joinLogPath(dfg, "$(runfile)_results.json"),"w")
println(fid, allStr)
close(fid)


## draw a second time


if parsed_args["batch_resolve"] && parsed_args["vis2d"]


pl = plotSLAM2D(dfg, drawhist=false)
pl |> PDF(joinLogPath(dfg,"fg_final_resolve.pdf"))

end

@show getLogPath(dfg)
