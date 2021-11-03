# test ScatterAlignPose2

using Test
using Images
using Caesar
using Distributions

# test plotting helper functions
using Gadfly


##
@testset "Test ScatterAlignPose2" begin
##

x = -10:0.1:10;
y = -10:0.1:10;

x = -10:0.1:10;

g = (x,y)->pdf(MvNormal([1.0;1.0]),[x;y]) + pdf(MvNormal([5;0.0],[1.0;1.0]),[x;y]) + pdf(MvNormal([0;5.0],[1.0;1.0]),[x;y])

img1 = zeros(length(x),length(y))
img2 = zeros(length(x),length(y))

for (i,x_) in enumerate(x), (j,y_) in enumerate(y)
  img1[i,j] = g(x_,y_)
  img2[i,j] = g(x_+1.0,y_)  # shifted by +1
end

sap = ScatterAlignPose2(img1, img2, (x,y); sample_count=150, bw=1.0)

# requires IIF at least v0.25.6
@test sample(sap.hgd1,1) isa Tuple
@test sample(sap.hgd2,10)[1] isa AbstractArray


## check packing and unpacking

psap = convert(PackedScatterAlignPose2, sap);
sap_ = convert(ScatterAlignPose2, psap);

@test sap.gridscale == sap_.gridscale
@test sap.sample_count == sap_.sample_count
@test sap.bw == sap_.bw

@test isapprox(sap.hgd1, sap_.hgd1, mmd_tol=1e-2)
@test isapprox(sap.hgd2, sap_.hgd2, mmd_tol=1e-2)


## use in graph

fg = initfg()

addVariable!(fg, :x0, Pose2)
addVariable!(fg, :x1, Pose2)

addFactor!(fg, [:x0;], PriorPose2(MvNormal([0.01;0.01;0.01])))
addFactor!(fg, [:x0;:x1], sap)

X1 = approxConvBelief(fg, :x0x1f1, :x1)

c1 = AMP.makeCoordsFromPoint(getManifold(Pose2), mean(X1))

@warn "skipping numerical check on ScatterAlignPose2 convolution test" c1
# @test isapprox( [-1;0], c1[1:2], atol=1.0 )
# @test isapprox( 0, c1[3], atol=0.5 )

## test plotting function

snt = overlayScatterMutate(sap; sample_count=50, bw=1.0);

plotScatterAlign(snt);

##
end