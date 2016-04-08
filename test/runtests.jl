using Ceasar
using Base.Test

global pass=false

include("datasets/VictoriaParkTypes.jl")
include("robots/VictoriaParkSystem.jl")
using VictoriaParkTypes

try
  T=40
  fg = emptyFactorGraph();
  idx = appendFactorGraph!(fg, d, f, toT=T, lcmode=:mmodal);
  tree = prepBatchTree!(fg, ordering=:qr);
  FG = FactorGraph[]
  TREE= BayesTree[]
  push!(FG,deepcopy(fg));
  push!(TREE,deepcopy(tree));
  for i in 1:2
     @time inferOverTree!(fg,tree, N=100);
     push!(FG,deepcopy(fg));
     push!(TREE,deepcopy(tree));
  end
 global pass=true
catch
  global pass=false
end

@test pass
