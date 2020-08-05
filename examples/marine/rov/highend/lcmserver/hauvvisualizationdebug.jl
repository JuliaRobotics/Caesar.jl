# test DrakeVisualizer

# for more low level interaction with the DraekVisualizer/Director, please see
# www.github.com/rdeits/DrakeVisualizer.jl

using Caesar, CoordinateTransformations, Rotations
using RoME, Distributions
using TransformUtils
using CloudGraphs
using IncrementalInference
# for illustration only
using GeometryTypes
using DrakeVisualizer
import DrakeVisualizer: Triad
import Caesar: prepcolordepthcloud!


 # @load "usercfg.jld"
include(joinpath(dirname(@__FILE__),"..","database","blandauthremote.jl"))
user_config = addrdict
user_config["session"] = "SESSHAUVDEV1"
backend_config, user_config = standardcloudgraphsetup(addrdict=user_config)


vis = startdefaultvisualization(draworigin=true)
# Launch the viewer application if it isn't running already:
# DrakeVisualizer.any_open_windows() || DrakeVisualizer.new_window();

# Mongo keys with BSONpointclouds, take from SESSHAUVDEV5
mkpc = Vector{AbstractString}(5)
mkpc[1] = "58f268f7b2ca1516ff509b82"
mkpc[2] = "58f268feb2ca1516ff509b84"
mkpc[3] = "58f26900b2ca1516ff509b86"
mkpc[4] = "58f26902b2ca1516ff509b88"
mkpc[5] = "58f26904b2ca1516ff509b8a"


wTx = Vector{AffineMap}(5)
wTx[1] = Translation(8.7481, 7.51, 4.5662) ∘ LinearMap(Quat(0.9939, -0.0280019, -0.00824962, -0.106355))
wTx[2] = Translation(9.43, 11.0879, 4.9) ∘ LinearMap(Quat(0.992387, 0.0575732, -0.00698004, -0.108645))
wTx[3] = Translation(9.94, 15.1484, 4.8792) ∘ LinearMap(Quat(0.997597, 0.0563353, -0.0115003, -0.0386678))
wTx[4] = Translation(9.93104, 14.5831, 5.8) ∘ LinearMap(Quat(0.997626, 0.0336346, -0.011705, -0.0589371))
wTx[5] = Translation(9.12, 10.1615, 5.84927) ∘ LinearMap(Quat(0.998351, 0.0362801, -0.00973871, -0.0434122))


for i in 1:5
  # set pose
  sym = Symbol("x$(i)")
  setgeometry!(vis[:debughauv][sym], Triad())
  settransform!(vis[:debughauv][sym],   wTx[i])
  # and a point cloud
  data = read_MongoData(backend_config, mkpc[i])
  pcarr = getPointCloudFromBSON(data)
  pc = prepcolordepthcloud!(i, pcarr)
  setgeometry!(vis[:debughauv][sym][:pointcloud], pc)
end

# >>> view.camera().SetViewUp([0,0,-1])
# >>> view.render()



# fetch and study the graph as built by server.jl

# setBackendWorkingSet!(fg.cg.neo4j.connection, user_config["session"])
fg = initfg(cloudgraph=backend_config, sessionname=user_config["session"])
fullLocalGraphCopy!(fg, reqbackendset=true)
writeGraphPdf(fg)
run(`evince fg.pdf`)

tree = wipeBuildNewTree!(fg,drawpdf=true)
@async run(`evince bt.pdf`)

# Juno.breakpoint("/home/dehann/.julia/v0.5/IncrementalInference/src/ApproxConv.jl", 144)

ret = inferOverTreeR!(fg, tree, N=100, dbg=true)
visualizeallposes!(vis, fg)


spyCliqMat(tree, :x2)
plotUpMsgsAtCliq(tree, :x1, :x2, marg=[6;3])


plotPriorsAtCliq(tree, :x1, :x5, dims=[1;2])
plotPriorsAtCliq(tree, :x1, :x5, dims=[6;3])
plotPriorsAtCliq(tree, :x1, :x5, dims=[4;5])





drawAllBinaryFactorEdges!(vis, backend_config, user_config["session"])


# drawLineBetween!(vis, fg,:x1,:x2, api=localapi , color=RGBA(0,1.0,1.0,0.5), scale=0.045 )
# drawAllOdometryEdges!(vis, fg, api=localapi)
# drawAllBinaryFactorEdges!(vis, fg)






X1 = getBelief(fg, :x1)
plotKDE(X1, dims=[1;2])
X2 = getBelief(fg, :x2)
plotKDE(X2, dims=[1;2])



pp, = localProduct(fg, :x2)
plotKDE(pp, dims=[1;2])



## Recalculate XYH
# change toolbox
wTx1 = convert(SE3, wTx[1])
wTx2 = convert(SE3, wTx[2])
wTx3 = convert(SE3, wTx[3])
wTx4 = convert(SE3, wTx[4])
wTx5 = convert(SE3, wTx[5])
# get rotation to world frame for local level
wEx1 = convert(Euler, wTx1.R);  wEx1.Y = 0.0;
wRlx1 = SE3(zeros(3), wEx1)
wEx2 = convert(Euler, wTx2.R);  wEx2.Y = 0.0;
wRlx2 = SE3(zeros(3), wEx2)
wEx3 = convert(Euler, wTx3.R);  wEx3.Y = 0.0;
wRlx3 = SE3(zeros(3), wEx3)
wEx4 = convert(Euler, wTx4.R);  wEx4.Y = 0.0;
wRlx4 = SE3(zeros(3), wEx4)
wEx5 = convert(Euler, wTx5.R);  wEx5.Y = 0.0;
wRlx5 = SE3(zeros(3), wEx5)
# Odometries
wRlx1Tx2 = wRlx1 * (wTx1\wTx2)
wRlx2Tx3 = wRlx2 * (wTx2\wTx3)
wRlx3Tx4 = wRlx3 * (wTx3\wTx4)
wRlx4Tx5 = wRlx4 * (wTx4\wTx5)

vEx1_2 = veeEuler(wRlx1Tx2)
vEx2_3 = veeEuler(wRlx2Tx3)
vEx3_4 = veeEuler(wRlx3Tx4)
vEx4_5 = veeEuler(wRlx4Tx5)

XYH1_2 = [vEx1_2[1], vEx1_2[2], vEx1_2[6]]
XYH2_3 = [vEx2_3[1], vEx2_3[2], vEx2_3[6]]
XYH3_4 = [vEx3_4[1], vEx3_4[2], vEx3_4[6]]
XYH4_5 = [vEx4_5[1], vEx4_5[2], vEx4_5[6]]

odo_xyh_12 = getfnctype(getVert(fg, fg.fIDs[:x1x2]))
odo_xyh_23 = getfnctype(getVert(fg, fg.fIDs[:x2x3]))
odo_xyh_34 = getfnctype(getVert(fg, fg.fIDs[:x3x4]))
odo_xyh_45 = getfnctype(getVert(fg, fg.fIDs[:x4x5]))

zpr2 = veeEuler(wTx2)[[3;4;5]]
zpr3 = veeEuler(wTx3)[[3;4;5]]
zpr4 = veeEuler(wTx4)[[3;4;5]]
zpr5 = veeEuler(wTx5)[[3;4;5]]


# compare differences, these should all be zero
@show odo_xyh_12.xyy.μ - XYH1_2
@show odo_xyh_23.xyy.μ - XYH2_3
@show odo_xyh_34.xyy.μ - XYH3_4
@show odo_xyh_45.xyy.μ - XYH4_5


# build a new factor graph from this info
N = 100
fgnew = initfg(cloudgraph=backend_config, sessionname="SESSHAUVCODED")


addVariable!(fgnew,:x1,N=N,labels=["POSE"], dims=6)
addFactor!(fgnew,[:x1], PriorPose3(MvNormal(veeEuler(wTx1), 0.0001*eye(6)) ) )
initVariable!(fgnew, :x1)
plotKDE(fgnew, :x1, dims=[1;2], title="$(veeEuler(wTx1))")


addVariable!(fgnew,:x2,N=N,labels=["POSE"], dims=6)
a,b = MvNormal(zpr2[2:3],0.001*eye(2)), Distributions.Normal(zpr2[1], 0.01)
addFactor!(fgnew,[:x2], PartialPriorRollPitchZ( a, b ) )
addFactor!(fgnew,[:x1,:x2], PartialPose3XYYaw(MvNormal(XYH1_2,0.001*eye(3)))  )
initVariable!(fgnew, :x2)
plotKDE(fgnew, :x2, dims=[6;3], title="$(veeEuler(wTx2))")

visualizeallposes!(vis, fgnew)

pp, parr, par, lb, infdim = localProduct(fgnew, :x2)
plotKDE(pp, dims=[6;3], title="$(veeEuler(wTx2))")
# plotLocalProduct(fgnew, :x2, dims=[6;3], api=localapi)

# addVariable!(fgnew,:x2,N=N,labels=["POSE"], dims=6)
# a,b = MvNormal(zpr2[2:3],0.001*eye(2)), Distributions.Normal(zpr2[1], 0.1)
# addFactor!(fgnew,[:x2], PartialPriorRollPitchZ( a, b ) )
# addFactor!(fgnew,[:x1,:x2], deepcopy(odo_xyh_12)  )
# initVariable!(fgnew, :x2)
# plotKDE(fgnew, :x2, dims=[1;2], title="$(veeEuler(wTx2))")


# addVariable!(fgnew,:x2,N=N,labels=["POSE"], dims=6)
# a,b = MvNormal(zpr2[2:3],0.001*eye(2)), Distributions.Normal(zpr2[1], 0.1)
# addFactor!(fgnew,[:x2], PartialPriorRollPitchZ( a, b ) )
# odo = MvNormal( veeEuler(wTx1\wTx2),0.001*eye(6))
# addFactor!(fgnew,[:x1,:x2], Pose3Pose3(odo)  )
# initVariable!(fgnew, :x2)
# plotKDE(fgnew, :x2, dims=[1;2], title="$(veeEuler(wTx2))")


addVariable!(fgnew,:x3,N=N,labels=["POSE"], dims=6)
a,b = MvNormal(zpr3[2:3],0.001*eye(2)), Distributions.Normal(zpr3[1], 0.1)
addFactor!(fgnew,[:x3], PartialPriorRollPitchZ( a, b ) )
addFactor!(fgnew,[:x2,:x3], PartialPose3XYYaw(MvNormal(XYH2_3,0.001*eye(3)))  )
initVariable!(fgnew, :x3)
plotKDE(fgnew, :x3, dims=[1;2], title="$(veeEuler(wTx3))")




visualizeallposes!(vis, fgnew)




addVariable!(fgnew,:x4,N=N,labels=["POSE"], dims=6)
a,b = MvNormal(zpr4[2:3],0.001*eye(2)), Distributions.Normal(zpr4[1], 0.1)
addFactor!(fgnew,[:x4], PartialPriorRollPitchZ( a, b ) )
addFactor!(fgnew,[:x3,:x4], PartialPose3XYYaw(MvNormal(XYH3_4,0.001*eye(3)))  )
initVariable!(fgnew, :x4)
plotKDE(fgnew, :x4, dims=[1;2], title="$(veeEuler(wTx4))")




addVariable!(fgnew,:x5,N=N,labels=["POSE"], dims=6)
a,b = MvNormal(zpr5[2:3],0.001*eye(2)), Distributions.Normal(zpr5[1], 0.1)
addFactor!(fgnew,[:x5], PartialPriorRollPitchZ( a, b ) )
addFactor!(fgnew,[:x4,:x5], PartialPose3XYYaw(MvNormal(XYH4_5,0.001*eye(3)))  )
initVariable!(fgnew, :x5)
plotKDE(fgnew, :x5, dims=[1;2], title="$(veeEuler(wTx5))")




visualizeallposes!(vis, fgnew)
solve(fgnew)


treenew = wipeBuildNewTree!(fgnew, drawpdf=true)
inferOverTreeR!(fgnew, treenew, N=100, dbg=true)


plotUpMsgsAtCliq(treenew, :x1, :x2, marg=[6;3])


plotPriorsAtCliq(treenew, :x1, :x5, dims=[1;2])
plotPriorsAtCliq(treenew, :x1, :x5, dims=[6;3])
plotPriorsAtCliq(treenew, :x1, :x5, dims=[4;5])

















#
