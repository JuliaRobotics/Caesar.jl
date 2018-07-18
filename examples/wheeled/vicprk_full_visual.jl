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
# fg = Caesar.initfg(sessionname=user_config["sessionId"], robotname=user_config["robotId"], cloudgraph=backend_config)
# deleteServerSession!(fg.cg, user_config["sessionId"])

symbolmatch(s::Symbol, st::AS) where {AS <: AbstractString} = ismatch(Regex("$(st)"), string(s))
symbolmatch(s::Symbol, st::Char) = symbolmatch(s, string(st))

landmarksymbol(i::Int) = Symbol("l$i")
landmarknumber(sy::Symbol) = parse(Int, string(sy)[2:end])

function addoccurance!(lmo::Dict{Symbol, Int}, lm::Symbol)
  lmo[lm] = haskey(lmo, lm) ? lmo[lm]+1 : 1
  nothing
end

"""
    $(SIGNATURES)

For multiple measurements::Dict{Int,T} with unique Int signature, add "LANDMARK" `;softtype=Point2` to the fg::FactorGraph.  Type T not used in this function.
"""
function findAddNewLandmarks!(fg::FactorGraph, lmoccur::Dict{Symbol, Int}, measurements::Dict{Int, T}; min_occurance::Int=2, softtype=Point2) where T
  # Count occurances of each landmark
  lmsyms = landmarksymbol.(collect(keys(measurements)))
  addoccurance!.(lmoccur, lmsyms)

  # add new landmarks not already in factor graph
  lmsinfg = ls(fg)[2]
  for lm in lmsyms
    if lmoccur[lm] >= min_occurance
      if !(lm in lmsinfg)
        # add a new landmark
        println("adding lm=$(lm)")
        addNode!(fg, lm, softtype, labels=["LANDMARK";])
      end
      # @show measurements[landmarknumber(lm)]
    end
  end
  nothing
end

function findAddLandmFactorsByPose!(fgl::FactorGraph, psym::Symbol, meas::Dict{Int, T};
                    range_sigma::Float64=1.0, bearing_sigma::Float64=0.05, N::Int=100) where T
  lmsyms = landmarksymbol.(collect(keys(meas)))
  posels = ls(fgl, psym)
  lmsinfg = ls(fgl)[2]
  lms = intersect(lmsyms, lmsinfg)
  for lm in lms
    btwfcts = intersect(ls(fgl, lm), posels)
    if length(btwfcts) == 0
      z_rb = meas[landmarknumber(lm)]
      println("addFactor! $(psym) -- $(lm), z_rb=$(z_rb)")
      pp = Pose2Point2BearingRange( Normal(z_rb[2],bearing_sigma),Normal(z_rb[1],range_sigma) )
      addFactor!(fg, [psym; lm], pp)
    end
  end
  nothing
end


function drive!(fgl::FactorGraph, idx::Int, odos::Dict, meas::Dict, lmoccurl::Dict;
                N::Int=100, min_occurance::Int=2 )
  prev, X, nextn = getLastPose2D(fgl)
  vp, fp = addOdoFG!(fgl, nextn, odos[idx][1:3], Podo, N=N)

  findAddNewLandmarks!(fgl, lmoccurl, meas[idx], min_occurance=min_occurance)

  # walk backwards
  donels = Symbol[]
  for i in 0:(min_occurance-1)
    @show psym = Symbol("x$(idx-i-1)") # off by one
    if idx-i > 0
      currmeas = meas[idx-i]
      findAddLandmFactorsByPose!(fgl, psym, currmeas, N=N)
    end
  end
  nothing
end

function donextframe!(fgl::FactorGraph, idx::Int, odos::Dict, meas::Dict, lmoccurl::Dict; N::Int=100)
  drive!(fgl, idx, odos, meas, lmoccurl, N=N)

  tree = wipeBuildNewTree!(fgl)
  inferOverTree!(fgl, tree,N=N)

  drawPosesLandms(fgl)
end


Podo=diagm([0.5;0.5;0.005])
lsrNoise=diagm([0.1;1.0])


# start factor graph
N=100
fg = Caesar.initfg()
lmoccur = Dict{Symbol, Int}()

# init pose
prevn = initFactorGraph!(fg, init=d[1][1:3])[1]


measurements = f[1]
findAddNewLandmarks!(fg, lmoccur, measurements)
# there might not be any landmarks yet
findAddLandmFactorsByPose!(fg, :x0, f[1], N=N)



# pbr = Pose2Point2BearingRange(Normal(br[1], cov[1,1]), Normal(br[2],  cov[2,2]))  #{Normal, Normal}
# f = addFactor!(fg, [vps;vlm], pbr, ready=ready, autoinit=true ) #[vps;vlm],


drive!(fg, 2, d, f, lmoccur, N=N)


ls(fg)


writeGraphPdf(fg)




tree = wipeBuildNewTree!(fg)
inferOverTree!(fg, tree,N=N)



drawPosesLandms(fg)



pl = donextframe!(fg, 3, d, f, lmoccur)

pl = donextframe!(fg, 4, d, f, lmoccur)

pl = donextframe!(fg, 5, d, f, lmoccur)

pl = donextframe!(fg, 6, d, f, lmoccur)

pl = donextframe!(fg, 7, d, f, lmoccur)

pl = donextframe!(fg, 8, d, f, lmoccur)





# on robot front-end acquistion

fg = initfg()


addNode!(fg, :x0, Pose2)
# odo
addNode!(fg, :x1, Pose2)
# odo
addNode!(fg, :x2, Pose2)
# odo
addNode!(fg, :x3, Pose2)





# somewhere else get data and detect loop closures



# somehow get all AT detections
# you get some kind of list/dict of AT detections per pose
for p in poses
  # p is symbol such as :x45
  for detection in dets[p]
    # detection has (id, homography)
    lmsyms = ls(fg)[2]
    atsym = Symbol("l$(detection.id)")
    if !(atsym in lmsyms)
      addNode!(fg, atsym, Point2(labels=["LANDMARK"; "APRILTAG"]))
    end

    # IOU reverse homography to range, bearing
    pp2 = Point2Point2BearingRange(...)
    addFactor!(fg, [p; atsym], pp2)
  end
end





















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
