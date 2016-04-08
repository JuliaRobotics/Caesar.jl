using Caesar, IncrementalInference, RoME, HDF5, JLD
using Base.Test

global pass=false
#
# include("VictoriaParkTypes.jl")
# include("VictoriaParkSystem.jl")
# using VictoriaParkTypes

# include("data/Vic120MM.jl")

d = jldopen("data/VicPrkBasic.jld", "r") do file
  read(file, "dBasic")
end
f = jldopen("data/VicPrkBasic.jld", "r") do file
  read(file, "fBasic")
end
MM = jldopen("data/VicPrkBasic.jld", "r") do file
  read(file, "MMBasic")
end

try
  T=20
  fg = emptyFactorGraph();
  idx = appendFactorGraph!(fg, d, f, toT=T, lcmode=:mmodal, MM=MM);
  tree = prepBatchTree!(fg, ordering=:qr);
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
  
  global pass=true
catch e
  global pass=false
  rethrow(e)
end

@test pass
