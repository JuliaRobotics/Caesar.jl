
using Test
using Caesar


##


@testset "test Pose2AprilTag4Corners" begin

##

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

# test construction of factor
apt4 = Pose2AprilTag4Corners(corners)

## test adding to a graph
fg = generateCanonicalFG_ZeroPose2()
addVariable!(fg, :tag17, Pose2)

atf = addFactor!(fg, [:x0;:tag17], apt4)


meas = freshSamples(apt4,2)

@test  meas isa Tuple
@test  meas[1] isa Array

##



pts = approxConv(fg, ls(fg,:tag17)[1], :tag17)


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

#



end


#