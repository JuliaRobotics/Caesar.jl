## recommnded usage:
# currently needs to load from a folder (code bug workaround)
# julia -O3 -p8 quickBatchSolve.jl ~/Downloads/saved.tar.gz

##

@show ARGS

using DistributedFactorGraphs # TODO remove this line after IIF v0.11.1
using RoME
@everywhere using RoME

fg = initfg()
loadDFG(ARGS[1], Main, fg)

dontMarginalizeVariablesAll!(fg)
# solve all but "drt" labels
vars = setdiff(ls(fg), ls(fg, r"drt"))
(x->setSolvable!(getVariable(fg, x), 1)).(vars)
factrs = setdiff(lsf(fg), lsf(fg, r"drt"))
(x->setSolvable!(getFactor(fg, x), 1)).(factrs)

initAll!(fg)

getSolverParams(fg).drawtree = true
getSolverParams(fg).maxincidence = 1000

@time solveTree!(fg);

savepath = splitpath(ARGS[1])
savename = split(savepath[end], '.')[1]

newfile = joinpath(savepath[1:end-1]...,savename*"_solve")
saveDFG(fg, newfile)

@show newfile*".tar.gz"
@show getLogPath(fg)

##







#
