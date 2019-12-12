# messing with the tree ordering

# using Revise

using Caesar, IncrementalInference
using DistributedFactorGraphs

using Combinatorics
using Suppressor


# import IncrementalInference: resetBuildTreeFromOrder!

filepath = "/media/dehann/temp2/caesar/2019-12-12T00:46:15.438/fg_before_x35"

fg = initfg()
loadDFG(filepath, Main, fg)




drawGraph(fg)

vo = getEliminationOrder(fg)

vo2 = reverse(vo)

tree = resetBuildTreeFromOrder!(fg, vo2)



drawTree(tree, show=true)



### what we want for brute force cost on tree ordering...

# - generate the next ordering
vo = nextInPermutation

# our own
tree = buildTreeFromOrdering!(vo)
getTreeCostBruteForce(tree; revision=1, kargs...)


# comparible to
getIncidenceMatrixCost_MD(vo)
getIncidenceMatrixCost_AMD(incidence wrt vo)
getIncidenceMatrixCost_COLAMD
getIncidenceMatrixCost_CCOLAMD












# using Revise

using Caesar, IncrementalInference
using DistributedFactorGraphs

using Combinatorics
using Suppressor


# import IncrementalInference: resetBuildTreeFromOrder!

filepath = "/media/dehann/temp2/caesar/2019-12-12T00:46:15.438/fg_before_x35"

fg = initfg()
loadDFG(filepath, Main, fg)










function main(fg::AbstractDFG; upto::Int=6, num::Int=100)

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

  vars = ls(fgs)

  vos = permutations(vars) |> collect
  svos = deepcopy(vos)

  svos = round.(Int, length(vos)*rand(num) )
  TREES = Dict{Int, BayesTree}()
  @suppress begin
    ind = 0
    for vo in vos[svos]
      ind += 1
      TREES[ind] = resetBuildTreeFromOrder!(fgs, vo)
    end
  end

  return TREES
end



@time trees = main(fg)






#
# vos = rand(1000)
# svos = round.(Int, length(vos)*rand(100) )
# vos[svos]

#
