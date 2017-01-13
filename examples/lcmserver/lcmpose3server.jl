
using Caesar, RoME
using TransformUtils
using Rotations, CoordinateTransformations
using PyLCM
using PyCall

lcmtpath = joinpath(dirname(@__FILE__),"lcmtypes")


# println("Compile LCM types")
# run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/rome_pose_node_t.lcm`)
# run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/rome_pose_pose_nh_t.lcm`)

println("Adding lcmtypes dir to Python path: $(lcmtpath)")
unshift!(PyVector(pyimport("sys")["path"]),lcmtpath)

@pyimport rome


# function whatsthere(channel, msgdata)
#   posenode = rome.pose_node_t[:decode](msgdata)
#   @show posenode[:utime]
#   nothing
# end


vc = startdefaultvisualization()
sleep(3.0)
rovt = loadmodel(:rov)
rovt(vc)

lastpose = SE3(0)

function lcmaddodo!(vc::VisualizationContainer, slam::SLAMWrapper, msgdata)
  posenode = rome.pose_node_t[:decode](msgdata)
  # @show posenode[:utime]
  tA = posenode[:mean][1:3]
  posenode[:mean][4:6]
  EA = Euler(posenode[:mean][6], posenode[:mean][5], posenode[:mean][4])
  q = TransformUtils.convert(Quaternion, EA)
  newpose = SE3(tA, EA)
  am = Translation(tA...) ∘ LinearMap(Quat(q.s,q.v...))
  # updaterealtime!(vc,:body, am)
  # visualizerealtime(vc)

  Dx = lastpose\newpose # get delta transform between poses

  rovt( vc, am )
  # change to using pose prior and interpose translation only
  v,f = addOdoFG!(slam, Pose3Pose3( Dx, diagm(posenode[:Cov]) )  )
  println("Added node $(v.label)")
  nothing
end




function runlistener(fl::Vector{Bool}, slam::SLAMWrapper, vc::VisualizationContainer) #slam::SLAMWrapper)

  gg = (channel, msg_data) -> lcmaddodo!(vc, slam, msg_data)

  print("Preparing LCM...")
  lc = LCM()
  subscribe(lc, "ROME_NODES", gg) #lcmaddodo!
  while fl[1]
    handle(lc)
  end
  println("done")
  nothing
end

function setupSLAMinDB()
  SLAMWrapper(initfg(sessionname="HAUV"), nothing, 0)
end




# am = Translation(0.0,0,0) ∘ LinearMap(Quat(0.0,1.0,0,0))
# updaterealtime!(vc,:body, am)
# visualizerealtime(vc)
slam = setupSLAMinDB()



flags = Bool[1]
flags[1] = true

@async runlistener(flags, slam, vc)
println("LCM listener running...")



# run this to stop the lcm listener loop
flags[1] = false




#
