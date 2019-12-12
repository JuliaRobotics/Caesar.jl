
using Distributed
addprocs(10)


# using Suppressor
using Combinatorics, Random, Dates
using Caesar, IncrementalInference, DistributedFactorGraphs
@everywhere using Caesar, IncrementalInference, DistributedFactorGraphs, Combinatorics, Random, Dates



# import IncrementalInference: resetBuildTreeFromOrder!

filepath = "/media/dehann/temp2/caesar/2019-12-12T00:46:15.438/fg_before_x35"

fg = initfg()
loadDFG(filepath, Main, fg)




function shrinkFactorGraph(fg; upto::Int=6)

  fgs = deepcopy(fg)

  delVars = filter(x->isSolvable(getVariable(fgs, x))==0, ls(fgs))
  todel = setdiff(lsf(fgs, solvable=0), lsf(fgs, solvable=1))
  delFcts = intersect(lsf(fgs), todel)
  allMags = filter(x->:MAGNETOMETER in getTags(getFactor(fgs, x)), lsfPriors(fgs) )
  union!(delFcts, filter(x->length(ls(fgs, x))==0, allMags) )

  union!(delVars, (ls(fgs, r"x\d") |> sortDFG)[upto:end])
  union!(delFcts, map(x->ls(fgs, x), delVars)...)

  map(x->deleteFactor!(fgs, x), delFcts)
  map(x->deleteVariable!(fgs, x), delVars)

  fgs
end




@everywhere function main(fgs::AbstractDFG,  rseed; num::Int=100)

  Random.seed!(rseed)

  vars = ls(fgs) |> sortDFG

  vos = permutations(vars) |> collect
  svos = deepcopy(vos)

  svos = round.(Int, length(vos)*rand(num) )

  svos[svos .== 0] .= 1
  TREES = Dict{Int, Tuple{BayesTree, Vector{Symbol}}}()
  # @suppress begin
    ind = 0
    for vo in vos[svos]
      ind += 1
      TREES[svos[ind]] = (resetBuildTreeFromOrder!(fgs, vo), vo)
    end
  # end

  return TREES
end


fgs = shrinkFactorGraph(fg, upto=8)


@time trees = main(fgs, num=10)


wp = WorkerPool([2:nprocs();])
FUT = Future[]


for i in 1:100
  push!(FUT, remotecall(main, wp, fgs, round(Int, 10000*rand()) ))
end

TRE = map(x->fetch(x), FUT)


MANYTREES = Dict{Int, Tuple{BayesTree, Vector{Symbol}}}()
for DTRE in TRE
  try
    for (id,tr) in DTRE
      @show id
      MANYTREES[id]=tr
    end
  catch
      #
  end
end


MANYTREES

## how easy to save tree




## How to compute the cost for each of these trees
# i) max_depth * max_clique^(alpha)

tree.cliques








function getTreeNumParentsTotal(tree)
  for cliq in tree.bt.cliques
end


function getTreeCostBruteForce_01(tree,
                                  vo;
                                  alpha::Float64=1.0 )
  #
  maxdepth = map(x->getCliqDepth(tree, x)+1, vo) |> maximum
  maxdim = length.(map(x->getCliqAllVarSyms(fgs,getCliq(tree, x)), vo)) |> maximum
  # drawTree(tree, show=true)

  return maxdepth * (maxdim^alpha)
end

cliqid = 1

drawTree(tree, show=true)

function getTreeCostBruteForce_02(tree,
                                  vo;
                                  alpha::Float64=1.0 )
  #
  # frontal and number of children
  ARR = Tuple{Symbol, Int}[]
  for (cliqid, vex) in tree.cliques
    afrtl = getCliqFrontalVarIds(tree.cliques[cliqid])[1]
    numch = length(getChildren(tree, afrtl))
    push!(ARR, (afrtl, numch))
  end

  numParents = filter(x->0<x[2], ARR) |> length
  totalNumChildren = (x->x[2]).(ARR) |> sum

  return getTreeCostBruteForce_01(tree, vo, alpha=alpha)/(totalNumChildren/numParents)
end



BFCOST_01 = Dict{Int, Float64}()
BFCOST_02 = Dict{Int, Float64}()

for (id, treevo) = MANYTREES
  BFCOST_01[id] = getTreeCostBruteForce_01(treevo...)
  BFCOST_02[id] = getTreeCostBruteForce_02(treevo...)
end


using Gadfly
Gadfly.set_default_plot_size(35cm,20cm)


cost = BFCOST_01 |> values |> collect
costkeys = BFCOST_01 |> keys |> collect

Gadfly.plot(x=cost, Geom.histogram)


cost = BFCOST_02 |> values |> collect
costkeys = BFCOST_02 |> keys |> collect

Gadfly.plot(x=cost, Geom.histogram)


minimum(cost)

goodies = findall(x->x .< 1.201, cost)


drawTree(MANYTREES[costkeys[goodies[4]]][1], show=true)


#
