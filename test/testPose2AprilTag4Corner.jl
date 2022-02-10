
using Test
using Caesar
using AprilTags
using Optim
using Statistics
using Manifolds

using FileIO

# using ImageView

import DistributedFactorGraphs: AbstractRelative

##


@testset "test Pose2AprilTag4Corners" begin

##

tag_id = 0
arr = [
  0.5533489436587586;
  -0.14817352826667163;
  5.505868642129518;
  -0.19093946817093482;
  0.5594280845251081;
  3.275940643894491;
  -0.0006414676297225159;
  -0.0009483559382987803;
  0.023125003286657522;
]

homog = reshape(arr, 3, 3)

# pixel values
x1 = 210.54965209961784
x2 = 274.48315429688637
x3 = 264.912078857414
x4 = 206.38191223143477
y1 = 176.4522705078025
y2 = 169.23143005371946
y3 = 107.78357696534272
y4 = 117.63999176024579


corners = ((x1,y1),(x2,y2),(x3,y3),(x4,y4))

##

f_w, c_w, c_h = 200.0, 180.0, 120.0
f_h = f_w

pose = tagOrthogonalIteration(corners,homog, f_w, f_h, c_w, c_h, taglength=0.035)


##

# test construction of factor
apt4 = Pose2AprilTag4Corners(corners=corners, homography=homog, f_width=f_w, c_width=c_w, c_height=c_h)


## test adding to a graph

fg = generateCanonicalFG_ZeroPose(varType=Pose2)
addVariable!(fg, :tag17, Pose2)

atf = addFactor!(fg, [:x0;:tag17], apt4)

meas = sampleFactor(IIF._getCCW(atf),2)
# meas = sampleFactor(fg, DFG.getLabel(atf),2)

@error "restore type checking for AprilTags4Corners Factor"
@test  meas isa Vector
@test  meas[1] isa ProductRepr # TBD likely to change to new Manifolds.jl type

##


pts = approxConv(fg, DFG.ls(fg,:tag17)[1], :tag17)

@test 1 < length(pts)
@test length(pts[1].parts[1]) == 2
@test size(pts[1].parts[2]) == (2,2)


## test packing of factor

pf = DFG.packFactor(fg, atf)
uf = DFG.unpackFactor(fg, pf)

uf4 = getFactorType(uf)

@test isapprox(apt4.corners[1][1], uf4.corners[1][1], atol=1e-12)
@test isapprox(apt4.corners[2][1], uf4.corners[2][1], atol=1e-12)
@test isapprox(apt4.corners[3][1], uf4.corners[3][1], atol=1e-12)
@test isapprox(apt4.corners[4][1], uf4.corners[4][1], atol=1e-12)

@test isapprox(apt4.corners[1][2], uf4.corners[1][2], atol=1e-12)
@test isapprox(apt4.corners[2][2], uf4.corners[2][2], atol=1e-12)
@test isapprox(apt4.corners[3][2], uf4.corners[3][2], atol=1e-12)
@test isapprox(apt4.corners[4][2], uf4.corners[4][2], atol=1e-12)

@test norm( apt4.homography - uf4.homography ) < 1e-6

@test norm( apt4.K - uf4.K ) < 1e-6

@test norm( apt4.taglength - uf4.taglength ) < 1e-6

@test apt4.id == uf4.id 


## test factor graph solution


solveTree!(fg);


## test saving to file and back

saveDFG("/tmp/atagtest", fg)
fg_ = loadDFG("/tmp/atagtest")
Base.rm("/tmp/atagtest.tar.gz")

uf4_ = getFactorType(fg_, DFG.ls(fg_, :tag17)[1])

@test norm( apt4.homography - uf4_.homography ) < 1e-6
@test norm( apt4.K - uf4_.K ) < 1e-6
@test norm( apt4.taglength - uf4_.taglength ) < 1e-6
@test apt4.id == uf4_.id 


## Test deconvolution

pred, meas = approxDeconv(fg, DFG.ls(fg, :tag17)[1])

M = Manifolds.SpecialEuclidean(2)
pred_ = exp.(Ref(M), Ref(identity_element(M)), pred)
meas_ = exp.(Ref(M), Ref(identity_element(M)), meas)

@test isapprox( M, mean(M,pred_),  mean(M, meas_), atol=0.5)


## test preimage search ("SLAM-wise calibration")


# in reality we'd do this over section of the graph for each sample
# FIXME this must be moved up into IncrementalInference
function _solveFactorPreimage(fct::Union{<:AbstractPrior, <:AbstractRelative}, 
                              meas_; # this is a SpecialEuclidean(2) tangent vector type, ProductRepr or newer 
                              method=BFGS(),
                              regularize::Real=0 )
  #

  minObj = 0 == regularize ? (x) -> fct.preimage[1](meas_, x) : (x) -> fct.preimage[1](meas_, x) + regularize*sum((x-fct.preimage[2]).^2)

  residual = Optim.optimize(minObj, fct.preimage[2], method )
  residual.minimizer
end


fct = getFactorType(fg, :x0tag17f1)

preImgs = zeros(5,10)


for i in 1:10
  println("finding preimage $i") 
  preImgs[:,i] .= _solveFactorPreimage(fct, pred[i], regularize=0.001)

  # residual = Optim.optimize((x) -> fct.preimage[1](meas[1][:,i], x), fct.preimage[2], BFGS())
  # store sample
  # preImgs[:,i] .= residual.minimizer
end

@show means = Statistics.mean(preImgs, dims=2)[:]
@show preimgSolve = means - fct.preimage[2]
@test sum(abs.(preimgSolve) .< [10;10;10;10;1]) == 5

##


end



##

@testset "test Pose2AprilTag4Corners conventions" begin

##

calibfile = joinpath(pkgdir(AprilTags),"data/CameraCalibration/taggridphoto.jpg")

cimg = load(calibfile)

# imshow(cimg)

detector = AprilTagDetector()
tags = deepcopy(detector(cimg))
freeDetector!(detector)


# nearby calibration
f_width = 3370.4878918701756 + 5
f_height = 3352.8348099534364 + 5
c_width = 2005.641610450976  + 5
c_height = 1494.8282013012076 + 5
# the physical size of the tag
taglength = 0.0315


##


ARR = Pose2AprilTag4Corners.(tags, f_width=f_width, f_height=f_height, c_width=c_width, c_height=c_height, taglength=taglength)

# make sure the detections are in the same order as the tag ids
@test ((n->(ARR[n].id == n)).(1:length(ARR)) |> sum) == length(ARR)

##

# check a tag on the left is actually on the left according to 
# body frame: fwd-lft-up <==> x-y-z
# distance x should show the tag roughly 42cm ahead of the camera body frame
@test 0.4 < ARR[3].Zij.Z.μ[1] < 0.5
@test 0.2 < ARR[3].Zij.Z.μ[2]
@test abs(ARR[3].Zij.Z.μ[3]) < 0.1

# tag on the right should be on the right
@test 0.4 < ARR[38].Zij.Z.μ[1] < 0.5
@test ARR[38].Zij.Z.μ[2] < -0.2
@test abs(ARR[38].Zij.Z.μ[3]) < 0.1

# check a tag in the center should be near the center
@test 0.4 < ARR[18].Zij.Z.μ[1] < 0.5
@test abs(ARR[18].Zij.Z.μ[2]) < 0.1
@test abs(ARR[18].Zij.Z.μ[3]) < 0.1

# check vertical-only displacement
@test 0.4 < ARR[1].Zij.Z.μ[1] < 0.5
@test 0.15 < ARR[1].Zij.Z.μ[2] < 0.3
@test abs(ARR[1].Zij.Z.μ[3]) < 0.1

@test 0.4 < ARR[5].Zij.Z.μ[1] < 0.5
@test 0.15 < ARR[5].Zij.Z.μ[2] < 0.3
@test abs(ARR[5].Zij.Z.μ[3]) < 0.1


## drawing test to ensure the functions are working


for tag in tags
  apt4 = Pose2AprilTag4Corners(corners=tag.p, homography=tag.H, f_width=f_width, c_width=c_width, c_height=c_height)
  drawBearingLinesAprilTags!( cimg, apt4,
                              f_width=f_width, c_width=c_width, taglength=taglength);
  #
end


##

end




## check the grid calibration cost function is working
## NOTICE this test has moved up into AprilTags.jl itself

# IMGS = Vector{typeof(cimg)}()
# push!(IMGS, cimg)

# tags_ = Vector{AprilTag}[]
# push!(tags_, tags)

# obj = (f_width, f_height, c_width, c_height) -> AprilTags.calcCalibResidualAprilTags!( IMGS, tags_, taglength=taglength, f_width=f_width, f_height=f_height, c_width=c_width, c_height=c_height )[1]
# obj_ = (fcwh) -> obj(fcwh...)

##

# check that it works
# obj_([f_width, f_height, c_width, c_height])

##

# start with any available parameters, this is a limited run just to confirm the code is functional
# for a full calibration run, the solver should be allowed to perform many more iterations
# result = optimize(obj_, [f_width; f_height ;c_width ;c_height ], BFGS(), Optim.Options(iterations = 5) )

##


#