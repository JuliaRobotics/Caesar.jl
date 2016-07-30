# call with
# ARGS[1] = T
# ARGS[2] = frac err
# ARGS[3] = nameOutput
# ARGS[4] = port1
# ARGS[5] = port2
@show ARGS

using Caesar, IncrementalInference, RoME, HDF5, JLD
d = jldopen("data/VicPrkBasic.jld", "r") do file
  read(file, "dBasic")
end
f = jldopen("data/VicPrkBasic.jld", "r") do file
  read(file, "fBasic")
end
# MM = jldopen("test/data/VicPrkBasic.jld", "r") do file
#   read(file, "MMBasic")
# end
include("data/Vic120MM.jl")
MMr = MMwithRedirects(MM);
frac = parse(Float64,ARGS[2])
MMe = MMRandErrs(MMr,frac=frac)
T=parse(Int,ARGS[1])

isamdict, fgu = doISAMSolve(d,f,toT=T, savejld=false, MM=MMe, retfg=true, port1=parse(Int,ARGS[4]),port2=parse(Int,ARGS[5]))

# isamdict = jldopen("data/fgWithISAMrefERR01.jld", "r") do file
#   read(file, "isamdict")
# end
# fgu = jldopen("data/fgWithISAMrefERR01.jld", "r") do file
#   read(file, "fgu")
# end

fg = emptyFactorGraph();
idx = appendFactorGraph!(fg, d, f, toT=T, lcmode=:mmodal, MM=MMe);
tree = prepBatchTree!(fg, ordering=:qr,drawpdf=true);
inferOverTree!(fg,tree, N=100);
fg1 = deepcopy(fg)
@save "results/fgT$(T)_$(ARGS[3])_perErr$(ARGS[2]).jld" fg1 isamdict fgu MM MMr MMe T frac

tic()
inferOverTree!(fg,tree, N=100);
comptime = toc()
fg2 = deepcopy(fg)
@save "results/fgT$(T)_$(ARGS[3])_perErr$(ARGS[2]).jld" fg1 fg2 isamdict fgu MM MMr MMe T frac comptime

tic()
inferOverTree!(fg,tree, N=100);
comptime = toc()
fg3 = fg
@save "results/fgT$(T)_$(ARGS[3])_perErr$(ARGS[2]).jld" fg1 fg2 fg3 isamdict fgu MM MMr MMe T frac comptime
