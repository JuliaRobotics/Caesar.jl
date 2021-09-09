
using Distributed
using ArgParse
using RoME, IncrementalInference, DistributedFactorGraphs
@everywhere using RoME, IncrementalInference, DistributedFactorGraphs


function parse_commandline()
    s = ArgParseSettings()

    @add_arg_table! s begin
        "files"
            help = "Txt describing Which .tar.gz to resolve"
            required = true
    end

    return parse_args(s)
end

pargs = parse_commandline()

fid = open(pargs["files"], "r")
loadFiles = readlines(fid)
close(fid)

for thefile in loadFiles

fg = initfg()
loadDFG(thefile, Main, fg)
# loadDFG(joinpath(@__DIR__,"fg_x37.tar.gz"), Main, fg)

dontMarginalizeVariablesAll!(fg)

Base.mv(thefile, thefile*"_moved", force=true)
try
  @show imgfile = split(splitpath(thefile)[end], '.')[1]
  Base.mv(joinpath("background", imgfile*".png"), joinpath("background", "old", imgfile*".png"), force=true)
catch
  #
end

# solve only last segment
vars = ls(fg, r"x\d", solvable=1) |> sortDFG
nrSolved = floor(Int, length(vars)/10)*10+1
nrSt = maximum([nrSolved-50, 1])

vars = vars[nrSt:nrSolved]
union!(vars, [:l1])

dontSolve = setdiff(ls(fg), vars)

(x->setSolvable!(fg, x, 0)).(dontSolve)




## Solve and look again

getSolverParams(fg).drawtree=true

tree = solveTree!(fg)


##

## resave with fixed lag?
# defaultFixedLagOnTree!(fg,30,limitfixeddown=false)

saveDFG(fg, thefile)

end

#
#
# plotVariableBeliefs(sfg, r"x\d")[1]
#


#
