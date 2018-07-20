# test assignment problem

using DocStringExtensions
using Distributions
using KernelDensityEstimate, KernelDensityEstimatePlotting
using Munkres
using TransformUtils

using Optim

# new cost
function S_min_kld(p::BallTreeDensity,q::BallTreeDensity)
  minimum(abs.([kld(p,q), kld(q,p)]))
end

objCost(cst::Array{Float64,2}, asgn::Vector{Int}) = sum(Float64[cst[i,j] for (i,j) in enumerate(asgn[asgn.>0])])




N=100

noiseOld = diagm([0.25;0.25].^2)
noiseNex = diagm([1.0;1.0].^2)

Tnoise = [randn(2); 0.1*randn()]
Tnoise = [0.5; 0.7; -0.1]

# old and new positions
old = [0.0 0 1; 1 1 1; 0 3 1; 0 4 1]'
nex = SE2(Tnoise)*old

# occlusion plus new
old2 = [0.0 0 1; 1 1 1; 2 3 1; 0 4 1]'
nex2 = SE2(Tnoise)*old2


OLD = [kde!(rand(MvNormal(old[1:2,1], noiseOld),N));
       kde!(rand(MvNormal(old[1:2,2], noiseOld),N));
       kde!(rand(MvNormal(old[1:2,3], noiseOld),N));
       kde!(rand(MvNormal(old[1:2,4], noiseOld),N)) ]

NEX = [kde!(rand(MvNormal(nex[1:2,1], noiseNex),N));
       kde!(rand(MvNormal(nex[1:2,2], noiseNex),N));
       kde!(rand(MvNormal(nex[1:2,3], noiseNex),N));
       kde!(rand(MvNormal(nex[1:2,4], noiseNex),N)) ]

NEX2 = [kde!(rand(MvNormal(nex2[1:2,1], noiseNex),N));
        kde!(rand(MvNormal(nex2[1:2,2], noiseNex),N));
        kde!(rand(MvNormal(nex2[1:2,3], noiseNex),N));
        kde!(rand(MvNormal(nex2[1:2,4], noiseNex),N)) ]


#
plotKDE(OLD, dims=[1;2], levels=2, c=["cyan"])
#
plotKDE(NEX2, dims=[1;2], levels=2, c=["red"])


"""
    $(SIGNATURES)

Return cost matrix with old elements as rows and new elements being assigned as columns.
"""
function assignmentCost(OLDl, NEXl, z::Vector{Float64})
  cost = zeros(length(OLDl),length(NEXl))
  xTo = inv(SE2(z))
  pts = ones(3,100)
  for i in 1:4, j in 1:4
    pts[1:2,:] = getPoints(NEX[j])
    nPts = xTo*pts
    nK = kde!(nPts[1:2,:])
    cost[i,j] = S_min_kld(OLDl[i], nK)
  end
  cost
end

"""
    $(SIGNATURES)

Determine base cost for new landmarks.
"""
function getNewElemCost(OLDl, NEXl;quant=0.75, xTo=zeros(3))
  c = assignmentCost(OLDl, NEXl, xTo)
  nelemcost = Float64[quantile(c[:,i],quant) for i in 1:length(NEXl)]'
  # nelemcost = Base.mean(c,1)
  repmat(nelemcost, length(NEXl),1) - 0.5*diagm(nelemcost')
end

function findAssignmentCost(OLDl, NEXl, z::Vector{Float64}; costNew::Union{Void, Array{Float64,2}}=nothing, quant=0.75)
  costNew = costNew == nothing ? getNewElemCost(OLDl, NEXl;quant=0.75, xTo=zeros(3)) : costNew
  cost = assignmentCost(OLDl, NEXl, z)
  cost = [cost; costNew]
  comp = munkres(cost)
  comp, objCost(cost, comp), cost
end


ep = 1e-5

costnew=getNewElemCost(OLD, NEX2;quant=0.5, xTo=zeros(3))
a1,c1,C1 = findAssignmentCost(OLD, NEX2, [0.0;0.0;-0.0], costNew=costnew, quant=0.01)
a2,c2,C2 = findAssignmentCost(OLD, NEX2, [0.0;0.0;-0.0+ep])

dfdx = (c2-c1)/ep


# base cost for assigning existing landmarks only
costnew=getNewElemCost(OLD, NEX2, quant=0.53, xTo=zeros(3))
# setup lambda for optmization
gg(x) = findAssignmentCost(OLD, NEX2, x, costNew=costnew)[2]
# find optimum solution
opm = optimize(gg, zeros(3))
# recover optimal assignment
@show opm.minimizer
oAsgn,oCost,oCostMat = findAssignmentCost(OLD, NEX2, opm.minimizer, costNew=costnew)








# from Munkres.jl

cost = rand(4,4)
best = [3;4;1;2]
for (i,j) in enumerate(best); cost[i,j]=0; end
comp = munkres(cost)
@assert best == comp

objCost(cost, comp)





# trying stuff

S_min_kld(OLD[1], NEX[4])

pts = randn(2,100); pts[2,1:50] += 4
testd = kde!(pts)
S_min_kld(NEX[4], testd)

#
