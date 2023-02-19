# quick scatter align test

using Revise
using Caesar, Images
using GLMakie
using TensorCast

##

world_model = zeros(0,2)
offs = zeros(1,2)
for y in -9.5:1:9.5, i in 1:100
  offs[2] = y
  world_model = vcat(world_model, 0.05*randn(1,2) + offs)
end

m1 = deepcopy(world_model)[300:700,:]

world_model = world_model[shuffle(1:size(world_model,1)), :]
m1 = m1[shuffle(1:size(m1,1)),:]

##


scatter(world_model)


##

fg = initfg()

addVariable!(fg, :x0, Pose2)
addVariable!(fg, :x1, Pose2)

addFactor!(fg, [:x0], PriorPose2(MvNormal([0;0;0.], [1 0 0; 0 1 0; 0 0 0.1])))


@cast wm_[i][d] := world_model[i,d]
@cast m1_[i][d] := m1[i,d]

wm__ = collect.(wm_)
m1__ = collect.(m1_)

bw = [0.1;0.1]
cloud1=manikde!(Position2, wm__; bw)
cloud2=manikde!(Position2, m1__; bw)

sap = ScatterAlignPose2(;
  cloud1,
  cloud2,
  sample_count = 500
)

addFactor!(fg, [:x0; :x1], sap)

##

x1 = approxConv(fg, :x0x1f1, :x1)
xy = (s->s.x[1]).(x1)
scatter(hcat(xy...)' )


##


nt = Caesar.overlayScatter(sap)
plotScatterAlign(nt)

##

@cast pts[i,d] := pts_[i][d]
scatter(pts)

##