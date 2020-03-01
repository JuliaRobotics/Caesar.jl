## recommnded usage:
# currently needs to load from a folder (code bug workaround)
# julia -O3 -p8 quickBatchSolve.jl ~/Downloads/saved.tar.gz

##

@show ARGS

using RoME
@everywhere using RoME

fg = initfg()
loadDFG(ARGS[1], Main, fg)

getSolverParams(fg).drawtree = true
@time solveTree!(fg);

savepath = splitpath(ARGS[1])
savename = split(savepath[end], '.')[1]

saveDFG(fg, joinpath(savepath[1:end-1],savename*"_solve"))


##







#
