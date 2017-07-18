# Victoria Park using-server

using Caesar, HDF5, JLD
using RoME

# load all the model data
# d = odometry information
# f = laser scanner detections
# MM = multi-modal individual id references
# MMr = reworked to map to only one previous feature
# examplefolder, datafolder
include(joinpath(Pkg.dir("Caesar"),"examples","wheeled","loadVicPrkData.jl"))


include(joinpath(Pkg.dir("Caesar"),"examples","database","blandauthremote.jl"))
user_config["session"] = "SESSVICPRK_DANIEL"
backend_config, user_config = standardcloudgraphsetup(addrdict=user_config)



# Start new session
fg = Caesar.initfg(sessionname=user_config["session"], cloudgraph=backend_config)


# init pose
prevn = initFactorGraph!(fg, init=d[1][1:3])
Podo=diagm([0.5;0.5;0.005])
N=100
lcmode=:unimodal
lsrNoise=diagm([0.1;1.0])


for idx=2:40
  prev, X, nextn = getLastPose2D(fg)
  vp, fp = addOdoFG!(fg, nextn, d[idx][1:3], Podo, N=N, labels=["POSE"])
  # add landmarks
  addLandmarksFactoGraph!(fg, f, idx, prevn, nextn, lcmode=lcmode, lsrNoise=lsrNoise, N=N, MM=MM)
  prevn = nextn
  # if (idx%10==0)
  #    Solve
  #    tree = prepBatchTree!(fg, drawpdf=true);
  #   @time inferOverTree!(fg,tree, N=100);
  # end
end






# fetch a local copy
fullLocalGraphCopy!(fg)
drawPosesLandms(fg)




# Remove the new session from the server
deleteServerSession!(fg.cg, user_config["session"])
















#
