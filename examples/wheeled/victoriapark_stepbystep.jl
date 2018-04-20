# Victoria Park
addprocs(3)
using HDF5, JLD, Gadfly, Colors, Cairo
using KernelDensityEstimate, Distributions
using Caesar, IncrementalInference, RoME




# Evaluate the likelihood of a point on the marginal belief of some variable
# note the dimensions must match
function evalLikelihood(fg::FactorGraph, sym::Symbol, point::Vector{Float64})
  p = getVertKDE(fg, sym)
  Ndim(p) == length(point) ? nothing : error("point (dim=$(length(point))) must have same dimension as belief (dim=$(Ndim(p)))")
  evaluateDualTree(p, reshape(point,Ndim(p),1))[1]
end

# Evaluate the likelihood of an Array{2} of points on the marginal belief of some variable
# note the dimensions must match
function evalLikelihood(fg::FactorGraph, sym::Symbol, points::Array{Float64,2})
  p = getVertKDE(fg, sym)
  Ndim(p) == size(points,1) ? nothing : error("points (dim=$(size(points,1))) must have same dimension as belief (dim=$(Ndim(p)))")
  evaluateDualTree(p, (points))
end

function batchSolve(fgl)
  tree = prepBatchTree!(fgl, drawpdf=true);
  @time inferOverTree!(fgl,tree, N=100);
end




# load all the model data
# d = odometry information
# f = laser scanner detections
# MM = multi-modal individual id references
# MMr = reworked to map to only one previous feature
# examplefolder, datafolder
include(joinpath(dirname(@__FILE__),"loadVicPrkData.jl"))



# T=30 # 1400
# fg = Caesar.initfg();
# idx = appendFactorGraph!(fg, d, f, toT=T, lcmode=:unimodal, MM=MMr);


# Start with a fresh factor graph

# insert the first pose node in two dimensions, that is [x, y, theta]

# add prior information as a factor to X1 is found in d[1]...


# Go look at what this function,
#  function appendFactorGraph!(fg::FactorGraph,
# is doing and see if you can build the new pose functions line by line, similar to
# what you had done in the first ROV example.



# add new poses via odom



  # init pose
fg = Caesar.initfg()
prevn = initFactorGraph!(fg, init=d[1][1:3])
Podo=diagm([0.5;0.5;0.005])
N=100
lcmode=:unimodal
lsrNoise=diagm([0.1;1.0])


for idx=2:50
  prev, X, nextn = getLastPose2D(fg)
  vp, fp = addOdoFG!(fg, nextn, d[idx][1:3], Podo, N=N)
  # add landmarks
  addLandmarksFactoGraph!(fg, f, idx, prevn, nextn, lcmode=lcmode, lsrNoise=lsrNoise, N=N, MM=MM)
  prevn = nextn
  # if (idx%10==0)
  #    Solve
  #    tree = prepBatchTree!(fg, drawpdf=true);
  #   @time inferOverTree!(fg,tree, N=100);
  # end
end


pl=drawPosesLandms(fg)
draw(PDF("test.pdf",20cm,20cm),pl)


:l1
plotKDE(getVertKDE(fg,:l125))

b=getKDEMax(getVertKDE(fg,:l125))

evalLikelihood(fg, :l125 , b)

X,L = ls(fg)
d=0
t=[]
z=zeros(length(L)+1,length(L)+1)

j=1;
for l1 in L
  i=1
  for l2 in L
    b=getKDEMax(getVertKDE(fg, l1))
    c=evalLikelihood(fg, l2 , b)
    if ( c>0.002 && l1!=l2 )
      d+=1
      #Replace this with A delete and replace
      #what the hell should i do now
      addSoftEqualityPoint2D(fg, l1, l2 )

      push!(t,(l1,l2))
      z[i,j]=1
    elseif (l1==l2)
      z[i,j]=2
    end
    i+=1
  end
  j+=1
end
@show d
@show t
@show z
Gadfly.spy(z)


#tree = prepBatchTree!(fg, drawpdf=true);
#run(`evince bt.pdf`)
#@time inferOverTree!(fg,tree, N=100);

pl=drawPosesLandms(fg)
draw(PDF("test.pdf",20cm,20cm),pl)

# draw the factor graph in a different way
writeGraphPdf(fg)
@async run(`evince fg.pdf`)


batchSolve(fg)
pl1=drawPosesLandms(fg)
draw(PDF("after.pdf",20cm,20cm),pl1)




appendFactorGraph()
# you should have one node X1 with a solid dot prior factor attached
Graphs.plot(fg.g)

# you can also draw the result with plotting using
drawPoses(fg)

# # alternative
# vc = startdefaultvisualization()
# visualizeallposes!(vc, fg, drawlandms=false)


# now add odometry, using information from d[2] for relative constaint to X2
# ... add node here
# delta x,, delta y, delta yaw, *, *


# ...



# coninue adding poses up to pose X10
