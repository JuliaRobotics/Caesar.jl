using Caesar
using IncrementalInference, RoME
using HDF5, JLD
using Base.Test

println("[TEST] basics")

as = ArcPointsRangeSolve([0.0;1.0;0.0],[1.0/sqrt(2.0);1.0/sqrt(2.0);0.0],[1.0;0.0;0.0],1.0)
findaxiscenter!(as)
@show as.center, as.axis, as.angle
@test norm(as.center-[0.0;0.0;0.0]) < 1e-4
@test norm(as.axis-[0.0;0.0;-1.0]) < 1e-4
@test -1e-4 < as.angle-pi/2 < 1e-4


as = ArcPointsRangeSolve([2.0;0;0.0],[0.5;1.5;0.0],[-1.0;0;0.0],1.5)
findaxiscenter!(as)
@show as.center, as.axis, as.angle
@test norm(as.center-[0.5;0.0;0.0]) < 1e-4
@test norm(as.axis-[0.0;0.0;1.0]) < 1e-4
@test -1e-4 < as.angle-pi < 1e-4

as = ArcPointsRangeSolve([0;-1.0;0],[0;0.5;1.5],[0;2.0;0.0],1.5)
findaxiscenter!(as)
@show as.center, as.axis, as.angle
@test norm(as.center-[0;0.5;0]) < 1e-4
@test norm(as.axis-[-1.0;0.0;0.0]) < 1e-4
@test -1e-4 < as.angle-pi < 1e-4

println("[SUCCESS]")


println("[TEST] CloudGraphs API calls...")
if false
  println("[TEST] with CloudGraphs with local DB data layer (multicore)...")
  include("fourdoortestcloudgraph.jl")
  println("[SUCCESS]")
else
  warn("[NOT TESTING] CloudGraphs interface -- you must enable it here at Caesar/test/runtests.jl, and you'll need Neo4j and MongoDB installed.")
end


if false
# using VictoriaParkTypes

  using Caesar, IncrementalInference, RoME, HDF5, JLD, Gadfly, Colors, Cairo
  d = jldopen("test/data/VicPrkBasic.jld", "r") do file
    read(file, "dBasic")
  end
  f = jldopen("test/data/VicPrkBasic.jld", "r") do file
    read(file, "fBasic")
  end
  # MM = jldopen("test/data/VicPrkBasic.jld", "r") do file
  #   read(file, "MMBasic")
  # end
  include("test/data/Vic120MM.jl")
  MMr = MMwithRedirects(MM);
  isamdict = jldopen("test/data/fgWithISAMrefERR01.jld", "r") do file
    read(file, "isamdict")
  end
  fgu = jldopen("test/data/fgWithISAMrefERR01.jld", "r") do file
    read(file, "fgu")
  end
  T=40 # 1400
  fg = emptyFactorGraph();
  idx = appendFactorGraph!(fg, d, f, toT=T, lcmode=:mmodal, MM=MMr);
  tree = prepBatchTree!(fg, ordering=:qr,drawpdf=true);
  @time inferOverTree!(fg,tree, N=100);
  @save "fgT1400v09err.jld" fg

  # include("examples/dev/ISAMRemoteSolve.jl")
  T=1400
  gtvals = doISAMSolve(d,f,toT=T, savejld=true, MM=MMr)

  # @load "test/data/isamdict.jld"
  # @load "test/data/fgu.jld"

  # include("examples/dev/ISAMRemoteSolve.jl")

    T=1400
    fg = emptyFactorGraph();
    idx = appendFactorGraph!(fg, d, f, toT=T, lcmode=:mmodal, MM=MMr);
    tree = prepBatchTree!(fg, ordering=:qr,drawpdf=true);
    @time inferOverTree!(fg,tree, N=100);
    @save "fgT1400v06.jld" fg
    FG = FactorGraph[]
    TREE= BayesTree[]
    push!(FG,deepcopy(fg));
    push!(TREE,deepcopy(tree));
    for i in 1:2
       @time inferOverTreeR!(fg,tree, N=100);
       push!(FG,deepcopy(fg));
       push!(TREE,deepcopy(tree));
    end
    for i in 1:2
       @time inferOverTree!(fg,tree, N=100);
       push!(FG,deepcopy(fg));
       push!(TREE,deepcopy(tree));
    end

    # drawCompPosesLandm(fg,isamdict, fgu, lbls=false,drawunilm=false)
else
  warn("not running full Victoria park dataset now")
end
