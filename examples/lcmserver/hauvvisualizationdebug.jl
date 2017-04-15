# test DrakeVisualizer

# for more low level interaction with the DraekVisualizer/Director, please see
# www.github.com/rdeits/DrakeVisualizer.jl

using Caesar, CoordinateTransformations, Rotations
using CloudGraphs
# for illustration only
using GeometryTypes
# using ColorTypes: RGB
using DrakeVisualizer
import DrakeVisualizer: Triad
import Caesar: prepcolordepthcloud!


 # @load "usercfg.jld"
include(joinpath(dirname(@__FILE__),"..","database","blandauthremote.jl"))
user_config = addrdict
# user_config["session"] = "SESSHAUVDEV5"
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
  pc = prepcolordepthcloud!(pcarr)
  setgeometry!(vis[:debughauv][sym][:pointcloud], pc)
end
