# test assignment problem

using Distributions
using KernelDensityEstimate, KernelDensityEstimatePlotting
using Munkres
using TransformUtils

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

OLD = [kde!(rand(MvNormal(old[1:2,1], noiseOld),N));
       kde!(rand(MvNormal(old[1:2,2], noiseOld),N));
       kde!(rand(MvNormal(old[1:2,3], noiseOld),N));
       kde!(rand(MvNormal(old[1:2,4], noiseOld),N)) ]

NEX = [kde!(rand(MvNormal(nex[1:2,1], noiseNex),N));
       kde!(rand(MvNormal(nex[1:2,2], noiseNex),N));
       kde!(rand(MvNormal(nex[1:2,3], noiseNex),N));
       kde!(rand(MvNormal(nex[1:2,4], noiseNex),N)) ]




plotKDE(OLD, dims=[1;2], levels=2, c=["cyan"])

plotKDE(NEX, dims=[1;2], levels=2, c=["red"])



function findAssignmentCost(OLD, NEX, z::Vector{Float64})
  cost = zeros(4,4)
  xTo = inv(SE2(z))
  pts = ones(3,100)
  for i in 1:4, j in 1:4
    pts[1:2,:] = getPoints(NEX[j])
    nPts = xTo*pts
    nK = kde!(nPts[1:2,:])
    cost[i,j] = S_min_kld(OLD[i], nK)
  end
  comp = munkres(cost)
  comp, objCost(cost, comp)
end

ep = 1e-5

a1,c1 = findAssignmentCost(OLD, NEX, [0.0;0.0;-0.0])
a2,c2 = findAssignmentCost(OLD, NEX, [0.0;0.0;-0.0+ep])

dfdx = (c2-c1)/ep



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
