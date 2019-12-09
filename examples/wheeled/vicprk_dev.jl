# add more julia processes
nprocs() < 4 ? addprocs(4-nprocs()) : nothing

using IncrementalInference
using HDF5, JLD, Gadfly, Colors, Cairo
using KernelDensityEstimate, Distributions
using Caesar, RoME
using RoMEPlotting
using DocStringExtensions

## load all the model data
# d = odometry information
# f = laser scanner detections
# MM = multi-modal individual id references
# MMr = reworked to map to only one previous feature
# examplefolder, datafolder

include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","loadVicPrkData.jl"))


# include(joinpath(ENV["HOME"],"Documents","blandauthlocal.jl"))
# backend_config, user_config = standardcloudgraphsetup(addrdict=addrdict)
# # Start new session
# addrdict["sessionId"] = "VICPRK_VID"
# addrdict["robotId"] = "Ute"
# fg = initfg(sessionname=user_config["sessionId"], robotname=user_config["robotId"], cloudgraph=backend_config)
# deleteServerSession!(fg.cg, user_config["sessionId"])


include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","VicPrk_FrontEndUtils.jl"))







## Example Construction and pieces

Podo=diagm([0.5;0.5;0.005])
lsrNoise=diagm([0.1;1.0])


# start factor graph
N=100
fg = initfg()
lmoccur = Dict{Symbol, Int}()

# init pose
prevn = initFactorGraph!(fg, init=d[1][1:3])[1]


measurements = f[1]
findAddNewLandmarks!(fg, lmoccur, measurements)
# there might not be any landmarks yet
findAddLandmFactorsByPose!(fg, :x0, f[1], N=N)



# pbr = Pose2Point2BearingRange(Normal(br[1], cov[1,1]), Normal(br[2],  cov[2,2]))  #{Normal, Normal}
# f = addFactor!(fg, [vps;vlm], pbr, solvable=ready, autoinit=true ) #[vps;vlm],


drive!(fg, 2, d, f, lmoccur, Podo, N=N)


ls(fg)


writeGraphPdf(fg)



tree = wipeBuildNewTree!(fg, drawpdf=true)
inferOverTree!(fg, tree,N=N)

@async run(`evince bt.pdf`)

PL = Dict{Int, Any}()

PL[2] = drawPosesLandms(fg)


PL[3] = donextframe!(fg, 3, d, f, lmoccur, Podo, N=N)

PL[4] = donextframe!(fg, 4, d, f, lmoccur, Podo, N=N)

PL[5] = donextframe!(fg, 5, d, f, lmoccur, Podo, N=N)

PL[6] = donextframe!(fg, 6, d, f, lmoccur, Podo, N=N)

PL[7] = donextframe!(fg, 7, d, f, lmoccur, Podo, N=N)

PL[8] = donextframe!(fg, 8, d, f, lmoccur, Podo, N=N)

PL[9] = donextframe!(fg, 9, d, f, lmoccur, Podo, N=N)

PL[10] = donextframe!(fg, 10, d, f, lmoccur, Podo, N=N)

PL[11] = donextframe!(fg, 11, d, f, lmoccur, Podo, N=N)

PL[12] = donextframe!(fg, 12, d, f, lmoccur, Podo, N=N)

PL[13] = donextframe!(fg, 13, d, f, lmoccur, Podo, N=N)

PL[14] = donextframe!(fg, 14, d, f, lmoccur, Podo, N=N)

PL[15] = donextframe!(fg, 15, d, f, lmoccur, Podo, N=N)

PL[16] = donextframe!(fg, 16, d, f, lmoccur, Podo, N=N)

PL[17] = donextframe!(fg, 17, d, f, lmoccur, Podo, N=N)

PL[18] = donextframe!(fg, 18, d, f, lmoccur, Podo, N=N)

PL[19] = donextframe!(fg, 19, d, f, lmoccur, Podo, N=N)

PL[20] = donextframe!(fg, 20, d, f, lmoccur, Podo, N=N)

PL[21] = donextframe!(fg, 21, d, f, lmoccur, Podo, N=N)

PL[22] = donextframe!(fg, 22, d, f, lmoccur, Podo, N=N)

PL[23] = donextframe!(fg, 23, d, f, lmoccur, Podo, N=N)

# importall RoMEPlotting
# drawSubmaps(fg, [1; ], spread=5, m2hist=true)


# get almost smallest neighboring landmarks to last pose
nhbls = ls2(fg, ls(fg)[1][end])
nhbls = nhbls[symbolmatch.(nhbls, 'l')]
lmn = landmarknumber.(nhbls)
minwindow = round(Int, quantile(lmn,0.01, sorted=false))

# largest landmark number
lmmax = landmarknumber(ls(fg)[2][end])

drawSubmaps(fg, [1  minwindow-1; minwindow lmmax], m2hist=true)




# on robot front-end acquistion


















# Figure export folder
currdirtime = now()
imgdir = joinpath(ENV["HOME"], "Pictures", "vicprkimgs", "$(currdirtime)")
mkdir(imgdir)


lcmode=:mmodal


#build :# poses for the factorGraph
for idx=9:30
  prev, X, nextn = getLastPose2D(fg)
  vp, fp = addOdoFG!(fg, nextn, d[idx][1:3], Podo, N=N)
  # add landmarks
  addLandmarksFactorGraph!(fg, f, idx, prevn, nextn, lcmode=lcmode, lsrNoise=lsrNoise, N=N, MM=MM)
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

fg = initfg(sessionname=user_config["sessionId"], cloudgraph=backend_config) #, robotname=user_config["robotId"]
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
      #what should i do now
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
fg = initfg(sessionname=user_config["sessionId"], cloudgraph=backend_config)
fullLocalGraphCopy!(fg)
pl2=drawPosesLandms(fg)
draw(PDF("daniel.pdf",20cm,20cm),pl2)


# Remove the new session from the server
deleteServerSession!(fg.cg, user_config["sessionId"])















#
