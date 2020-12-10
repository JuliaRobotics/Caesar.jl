
## change working directory

cd(joinpath(ENV["HOME"],"Documents","debugging","racecarimgs","5000","easy"))

## Load packages

using FreeTypeAbstraction
using AprilTags
using Caesar
using Images, ImageView, ImageDraw
using FileIO
using LinearAlgebra
using Cairo, Fontconfig, RoMEPlotting
Gadfly.set_default_plot_size(25cm, 20cm)

## make multiprocess

using Distributed

addprocs(4)
using FreeTypeAbstraction, AprilTags, Images, ImageView, FileIO, Caesar
@everywhere using FreeTypeAbstraction, AprilTags, Images, ImageView, FileIO, Caesar


## which images

imgFiles = [
  "1_4cb1b54b-e297-41ac-8a81-f0ddab02dac2.png",
  "2_ddaf9899-4c6a-4182-a7ad-4cc9be6f127d.png",
  "3_077227d4-e6cd-4884-befd-a0d73a07c172.png",
  "4_7545b43e-0731-42ee-8b57-df03755abbd6.png",
  "5_50ce6f66-1546-4f90-a7e5-336e595f14a2.png",
  "6_7676be16-8f57-4a2e-ac2b-1528747213d1.png",
  "7_c7c38417-eecd-4e37-91fc-4628e065ac66.png",
  "8_e42c4cc8-25b1-4df9-8790-61f8212e9ca2.png"
]

## load images into memory

imgs = load.(imgFiles)

rows, cols = size(imgs[1])
c_width = cols/2
c_height = rows/2
# f_width = 500.0
# f_height = f_width
taglength = 0.172

# f_width = 496.70305287384673
# c_height = 311.6691866287029

f_width = 504.91911888349216
# c_height = 324.2630820675616

f_height = f_width

K = Caesar._defaultCameraCalib(f_width=f_width, f_height=f_height, c_width=c_width, c_height=c_height)[1:2,:]

## test to see

imshow(vcat(hcat(imgs[1:3]...),hcat(imgs[4:6]...)))


## Run AprilTag detector (app)


detector = AprilTagDetector()
allTags = deepcopy(detector.(imgs))


## init the factor graph

fg = generateCanonicalFG_ZeroPose2()

getSolverParams(fg).useMsgLikelihoods = true

#

posecount = 0
prevSym = :x0

for tags in allTags[1:5]
  # new variable in factor graph
  posecount += 1
  poseSym = Symbol("x$posecount")
  addVariable!(fg, poseSym, Pose2, tags=[:POSE;])

  # add basic odo relative
  addFactor!(fg, [prevSym; poseSym], Pose2Pose2(MvNormal(zeros(3),diagm([1.0;0.5;0.5]))), tags=[:ODOMETRY;])
  for tag in tags
    @show tag.id
    tagSym = Symbol("tag$(tag.id)")
    if !exists(fg, tagSym)
      addVariable!(fg, tagSym, Pose2, tags=[:APRILTAG;])
    end

    apt4 = Pose2AprilTag4Corners(corners=tag.p, homography=tag.H, f_width=f_width, c_width=c_width, c_height=c_height, taglength=taglength, id=tag.id)
    addFactor!(fg, [poseSym; tagSym], apt4, tags=[:SIGHTING;])
  end
  prevSym = poseSym
end


## solve the graph


for i in 1:1
  solveTree!(fg, storeOld=true);
end



## solving and looking at the subgraph

# listSolveKeys(fg)

plp = plotSLAM2DPoses(fg, contour=false, drawEllipse=true)

# development on plot landmarks

pll = plotSLAM2DLandmarks(fg, regexLandmark=r"tag", contour=false, drawEllipse=true)

#

union!(plp.layers, pll.layers)
# co = Coord.Cartesian(xmax=4)
# plp.coord = co
plp



##


pl = plotSLAM2D(fg, regexLandmark=r"tag", contour=false, drawEllipse=true)

##

freeDetector!(detector)


## dev


imshow( drawTags(imgs[1], K) )


## look at one sighting factor

pred, meas = manikde!.(approxDeconv(fg, :x1tag16f1), Pose2)

plotPose(Pose2(), [meas;pred], c=["red";"green"])


## draw back for validation about what is going on


imgs_ = deepcopy(imgs);

for i in 1:5
  @show fSym = intersect(lsf(fg, Pose2AprilTag4Corners), ls(fg, Symbol("x$i")))
  for fct in getFactorType.(fg, fSym)
    drawBearingLinesAprilTags!( imgs_[i], fct,
                                f_width=f_width, c_width=c_width, taglength=taglength);
    #
  end
end

imshow( vcat(hcat( imgs_[1:3]... ), hcat( imgs_[4:6]... )) )



##

# img = deepcopy(imgs[1])

# drawBearingLinesAprilTags!(img, f_width=f_width, c_height=c_height, taglength=taglength)


# drawBearingLinesAprilTags!(deepcopy(imgs[1]), allTags[1], f_width=f_width, c_width=c_width, taglength=taglength) |> imshow
# drawBearingLinesAprilTags!(deepcopy(imgs[1]), f_width=f_width, c_width=c_width, taglength=taglength) |> imshow

# drawBearingLinesAprilTags!(deepcopy(imgs[2]), f_width=f_width, c_width=c_width, taglength=taglength) |> imshow
# drawBearingLinesAprilTags!(deepcopy(imgs[3]), f_width=f_width, c_width=c_width, taglength=taglength) |> imshow
# drawBearingLinesAprilTags!(deepcopy(imgs[4]), f_width=f_width, c_width=c_width, taglength=taglength) |> imshow


##


@show getFactorType(fg, :x1tag10f1).Zij.z.μ

allTags[1][2].id
Pose2AprilTag4Corners( corners=allTags[1][2].p, homography=allTags[1][2].H, f_width=f_width, c_width=c_width, c_height=c_height, taglength=taglength).Zij.z.μ




##


ls(fg, :x4)



##

plotLocalProduct(fg, :x4, dims=[1;2])

plotLocalProduct(fg, :x5, dims=[1;2])


plotLocalProduct(fg, :tag10, dims=[1;2])
plotLocalProduct(fg, :tag10, dims=[3])


## Some development tests


# accept default cost function argument mapping `args = (x)->[x[1]; x[1]; c_width; x[2]; taglength]`
#   such that `cost([f_width; c_height])`
cost = generateCostAprilTagsPreimageCalib(fg, f_width=f_width, f_height=f_height, c_width=c_width, c_height=c_height)

cost([f_width; c_height])


##

using Optim

##


result = optimize(cost, [f_width; c_height], BFGS())

result.minimizer

f_width, c_width


##


PL = []
for sk in [Symbol("default_$i") for i in 0:5]
  push!(PL, plotSLAM2D(fg,xmin=-2,xmax=7,ymin=-3,ymax=3, solveKey=sk))
end


plimgs = convert.(Matrix{RGB}, PL);

writevideo("pl100.avi", plimgs, fps=5)

##





## build subgraph [:x0; .. :x3;]

varfct = setdiff( union( ls.(fg, [:x0, :x1, :x2, :x3])...),  [:x4, :x3x4f1] )
vars = union(ls.(fg, varfct)...)
union!(varfct, vars)

sfg = buildSubgraph(fg,varfct)

##

for i in 1:5
  solveTree!(sfg, storeOld=true);
end



## subgraph 2


sfg = buildSubgraph(fg, [:x4; :x5; :tag10; :tag16], 1)

ls(sfg, :tag16)


## freeze the tags

# setMarginalize!()


##