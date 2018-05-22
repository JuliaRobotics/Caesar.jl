# Victoria Park using-server
addprocs(3)
using IncrementalInference
using HDF5, JLD, Gadfly, Colors, Cairo
using KernelDensityEstimate, Distributions
using Caesar, RoME
using RoMEPlotting


## load all the model data
# d = odometry information
# f = laser scanner detections
# MM = multi-modal individual id references
# MMr = reworked to map to only one previous feature
# examplefolder, datafolder

function evalLikelihood(fg::FactorGraph, sym::Symbol, point::Vector{Float64})
  p = getVertKDE(fg, sym)
  Ndim(p) == length(point) ? nothing : error("point (dim=$(length(point))) must have same dimension as belief (dim=$(Ndim(p)))")
  evaluateDualTree(p, (point')')[1]
end

# Evaluate the likelihood of an Array{2} of points on the marginal belief of some variable
# note the dimensions must match
function evalLikelihood(fg::FactorGraph, sym::Symbol, points::Array{Float64,2})
  p = getVertKDE(fg, sym)
  Ndim(p) == size(points,1) ? nothing : error("points (dim=$(size(points,1))) must have same dimension as belief (dim=$(Ndim(p)))")
  evaluateDualTree(p, (points))
end



include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","loadVicPrkData.jl"))


include(joinpath(ENV["HOME"],"Documents","blandauthlocal.jl"))
backend_config, user_config = standardcloudgraphsetup(addrdict=addrdict)
# Start new session
addrdict["sessionId"] = "VICPRK_VID"
addrdict["robotId"] = "Ute"


# Figure export folder
currdirtime = now()
imgdir = joinpath(ENV["HOME"], "Pictures", "vicprkimgs", "$(currdirtime)")
mkdir(imgdir)


fg = Caesar.initfg(sessionname=user_config["sessionId"], robotname=user_config["robotId"], cloudgraph=backend_config)
deleteServerSession!(fg.cg, user_config["sessionId"])

# init pose
prevn = initFactorGraph!(fg, init=d[1][1:3])[1]
Podo=diagm([0.5;0.5;0.005])
N=100
lcmode=:mmodal
lsrNoise=diagm([0.1;1.0])


#build :# poses for the factorGraph
for idx=9:30
  prev, X, nextn = getLastPose2D(fg)
  vp, fp = addOdoFG!(fg, nextn, d[idx][1:3], Podo, N=N)
  # add landmarks
  addLandmarksFactoGraph!(fg, f, idx, prevn, nextn, lcmode=lcmode, lsrNoise=lsrNoise, N=N, MM=MM)
  prevn = nextn
  if (idx%10==0)
     # Solve
     tree = wipeBuildNewTree!(fg, drawpdf=false);
    @time inferOverTree!(fg,tree, N=100);
  end
  pl=drawPosesLandms(fg,window=(nextn,25))
  Gadfly.draw(PNG(joinpath(imgdir,"$(nextn).png"),20cm,20cm),pl)
end


0

# run(`ffmpeg -y -i x%d.png -threads 4 -vcodec libx264 -s 1920x1080 -b:v 2M -filter:v "setpts=10*PTS" /home/dehann/Videos/vicprk_test.mp4`)


# pl=drawPosesLandms(fg);
# Gadfly.draw(PDF("/tmp/before.pdf",20cm,20cm),pl)


# batchSolve(fg)
# on ssh terminal, run slamindb(iterations=1)


# fetch a local copy

fg = Caesar.initfg(sessionname=user_config["sessionId"], cloudgraph=backend_config) #, robotname=user_config["robotId"]
fullLocalGraphCopy!(fg)
# pl1=drawPosesLandms(fg)


pl1=drawPosesLandms(fg,window=(:x9,25))


Gadfly.draw(PDF("/tmp/after.pdf",20cm,20cm),pl1)

@async run(`evince /tmp/after.pdf`)

subLocalGraphCopy!(fg, string.([:x1,:x2,:x3,:x4,:x5]), neighbors=1)



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
#run Server Code
fg = Caesar.initfg(sessionname=user_config["sessionId"], cloudgraph=backend_config)
fullLocalGraphCopy!(fg)
pl2=drawPosesLandms(fg)
draw(PDF("daniel.pdf",20cm,20cm),pl2)


# Remove the new session from the server
deleteServerSession!(fg.cg, user_config["sessionId"])















#
