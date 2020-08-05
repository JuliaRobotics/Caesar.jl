
using Distributed
addprocs(8)


# using Suppressor
using Combinatorics, Random, Dates
using Caesar, IncrementalInference, DistributedFactorGraphs, RoME
@everywhere using Caesar, IncrementalInference, DistributedFactorGraphs, Combinatorics, Random, Dates

using Gadfly
Gadfly.set_default_plot_size(35cm,20cm)

using Statistics, Colors
using JLD2, FileIO

using SparseArrays
using SuiteSparse.CHOLMOD: SuiteSparse_long

# bring in ccolamd
include(normpath(Base.find_package("IncrementalInference"), "..", "ccolamd.jl"))


fgs = generateCanonicalFG_Kaess()

drawGraph(fgs)


## checking out ccolamd

ind, varsym, fctsym = getAdjacencyMatrixSparse(fgs)
vo_ccolamd = varsym[Ccolamd.ccolamd(ind)]


tree = resetBuildTreeFromOrder!(fgs, vo_ccolamd)
drawTree(tree, show=true)

# add a constraint
cons = zeros(SuiteSparse_long, length(ind.colptr) - 1)
findall(x->x==:x28, varsym)
cons[9] = 23


vo_ccolamd = varsym[Ccolamd.ccolamd(ind, cons)]

tree = resetBuildTreeFromOrder!(fgs, vo_ccolamd)
drawTree(tree, show=true)


## qr ordering -- should be AMD
vo_qr = getEliminationOrder(fgs)

tree = resetBuildTreeFromOrder!(fgs, vo_qr)

drawTree(tree, show=true)



## start modifying order


findall(x->x==:x28, vo_qr)

vo_qr_m1 = vo_qr[[1:16;18:end]]
push!(vo_qr_m1, :x28)

tree = resetBuildTreeFromOrder!(fgs, vo_qr_m1)
drawTree(tree, show=true)


nnzTreeTotal(tree)
nnzQR(Matrix(ind))

q,r,p = qr(Matrix(ind), Val(true))
r .= abs.(r)
r[1e-5 .< r] .= 1
Gadfly.spy(Matrix(r))
varsym

nnzTreeTotal(tree)
nnzQR(Matrix(ind))

## Doing it ourselves
p = [:l1;:l2;:x1;:x2;:x3]
pTheirs = [:l2;:l1;:x3;:x1;:x2]

ind, varsym, fctsym = getAdjacencyMatrixSparse(fgs)

ourP = map(y->findfirst(x->x==y,varsym), p)
ourP = map(y->findfirst(x->x==y,varsym), pTheirs)

A = Matrix(ind)

Ap = A[:,ourP]

q,r = qr(Ap, Val(false))



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


fgs = shrinkFactorGraph(fg, upto=9)
drawGraph(fgs, show=true)

# @time trees = main(fgs, 1234, num=10)


wp = WorkerPool([2:nprocs();])
FUT = Future[]


for i in 1:100
  push!(FUT, remotecall(main, wp, fgs, round(Int, 10000*rand()) ))
end

# FUT2 = deepcopy(FUT)

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

## okay save stuff




i=1

# MANYTREES2 = deepcopy(MANYTREES)
MANYTREES2b = deepcopy(MANYTREES)



for (id, trevo) in MANYTREES2b
  tre, vo = trevo
  for i in 1:length(tre.cliques)
    if  tre.cliques[i].attributes["data"] isa BayesTreeNodeData
      tre.cliques[i].attributes["data"] = convert(PackedBayesTreeNodeData, tre.cliques[i].attributes["data"])
    end
    # stree.cliques[i].attributes["data"] = convert(PackedBayesTreeNodeData, tree.cliques[i].attributes["data"])
  end
end


for (id, trevo) in MANYTREES2b
  MANYTREES2[id] = trevo
end

MANYTREES2



@save joinpath(ENV["HOME"],"Downloads","MANYTREES2.jld2") MANYTREES2

data = @load joinpath(ENV["HOME"],"Downloads","MANYTREES2.jld2") MANYTREES2





## How to compute the cost for each of these trees
# i) max_depth * max_clique^(alpha)




### how to get AMD cost proxy nnz
# frontalDim(dim) = dim==1 ? 1 : dim+frontalDim(dim-1)
frontalDim(dim) = dim == 1 ? 1 : (dim * (dim + 1))/2


function nnzClique(cliq)
    dimFrontal = length(getCliqFrontalVarIds(cliq))
    dimSeparator = length(getCliqSeparatorVarIds(cliq))
    return frontalDim(dimFrontal) + dimSeparator * dimFrontal
end

function nnzTreeTotal(tree)
  nnzTot = 0
  for (cliqid, cliq) in tree.cliques
    # afrtl = getCliqFrontalVarIds(tree.cliques[cliqid])[1]
    # cliq = getClique(tree, afrtl)
    nnzTot += nnzClique(cliq)
  end
  return nnzTot
end


function nnzQR(A::Matrix)
  q,r,p = qr(A, Val(true))
  r .= abs.(r)
  nz = 1e-5 .< r
  r[nz] .= 1
  sum(nz)
end




## own costing function

function getTreeCostBruteForce_01(tree;
                                  alpha::Float64=1.0 )
  #
  cliqs = tree.cliques |> values |> collect
  maxdepth = map(x->getCliqDepth(tree, x)+1, cliqs) |> maximum
  maxdim = length.(map(x->getCliqVarIdsAll(x), cliqs)) |> maximum
  # drawTree(tree, show=true)

  return maxdepth * (maxdim^alpha)
end


function getTreeCostBruteForce_02(tree;
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

  return getTreeCostBruteForce_01(tree, alpha=alpha)/(totalNumChildren/numParents)
end


# Int: ID of the variable order
ALLSCORES = Vector{Tuple{Int, Float64, Float64, Float64}}()

BFCOST_01 = Dict{Int, Float64}()
BFCOST_02 = Dict{Int, Float64}()
BFCOST_nnz = Dict{Int, Float64}()

for (id, treevo) = MANYTREES2
  BFCOST_01[id] = getTreeCostBruteForce_01(treevo[1])
  BFCOST_02[id] = getTreeCostBruteForce_02(treevo[1])
  BFCOST_nnz[id] = nnzTreeTotal(treevo[1])

  push!(ALLSCORES, (id, BFCOST_01[id], BFCOST_02[id], BFCOST_nnz[id]))
end

ALLSCORES

sid = (x->x[1]).(ALLSCORES)

@assert length(unique(sid)) == length(sid)


cost_01 = (x->(x[2])).(ALLSCORES)
cost_02 = (x->(x[3])).(ALLSCORES)
cost_nnz = (x->(x[4])).(ALLSCORES)


Gadfly.plot(x=cost_nnz, y=cost_02, Geom.point)

Gadfly.plot(x=cost_nnz, y=cost_02, Geom.hexbin)








cost_01


costkeys_01 = BFCOST_01 |> keys |> collect

Gadfly.plot(x=cost_01, Geom.histogram)


cost_02 = BFCOST_02 |> values |> collect
costkeys_02 = BFCOST_02 |> keys |> collect

Gadfly.plot(x=cost_02, Geom.histogram)


cost_nnz=[]
costkeys_nnz=[]

for (a,b) in BFCOST_nnz
  push!(costkeys_nnz,a)
  push!(cost_nnz,b)
end

# cost_nnz = BFCOST_nnz |> values |> collect
# costkeys_nnz = BFCOST_nnz |> keys |> collect

pl_nnz = Gadfly.plot(x=cost_nnz, Geom.histogram)




goodies_nnz = findall(x->x .<= minimum(cost_nnz), cost_nnz)


tree, vo = MANYTREES2[costkeys_nnz[goodies_nnz[4]]]

drawTree(tree, show=true)

getTreeCostBruteForce_02(tree, vo)
maximum(cost_02)



## what is the intersect between good AMDs and good brute force'es




goodies_02 = findall(x->x .<= quantile(cost_02, 0.05), cost_02)
# goodies_02 = findall(x->x .<= minimum(cost_02), cost_02)

crosscost = cost_nnz[goodies_02]

pl_cc = Gadfly.plot(x=crosscost, Geom.histogram, Theme(default_color=colorant"red"))

union!(pl_nnz.layers, pl_cc.layers)

pl_nnz


CROSSTREES = MANYTREES[costkeys_nnz[goodies] ]


## filter goodies_02 for best and worst cost_nnz


filter(x->cost_nnz[x]==minimum(cost_nnz[goodies_02]), goodies_02)

453
524
1043

costkeys_02[453]
costkeys_nnz[453]
cost_02[453]
cost_nnz[453]

MANYTREES2[costkeys_02[453]][1]



drawTree(MANYTREES2[costkeys_02[453]][1], show=true)
getTreeCostBruteForce_02(MANYTREES2[costkeys_02[453]]...)

nnzTreeTotal(MANYTREES2[costkeys_02[453]][1])



using DataFrames

df = vcat(
DataFrame(
x=crosscost,
color="cross"
),
DataFrame(
 x=cost_nnz,
 color="nnz"
),
)

Gadfly.plot(df, Geom.histogram, x=:x, color=:color)


##



tree



#
