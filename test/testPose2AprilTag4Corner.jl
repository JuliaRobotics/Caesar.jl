
using Test
using Caesar
using AprilTags

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

pose = tagOrthogonalIteration(corners,homog,200.0,200.0,180.0,120.0,taglength=0.035)


##

# test construction of factor
apt4 = Pose2AprilTag4Corners(corners=corners, homography=homog, cx=180, cy=120, fx=300, fy=300)


## test adding to a graph
fg = generateCanonicalFG_ZeroPose2()
addVariable!(fg, :tag17, Pose2)

atf = addFactor!(fg, [:x0;:tag17], apt4)


meas = freshSamples(apt4,2)

@test  meas isa Tuple
@test  meas[1] isa Array

##



pts = approxConv(fg, DFG.ls(fg,:tag17)[1], :tag17)


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



##



end


#