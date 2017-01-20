
addprocs(6)

using Caesar, RoME
using TransformUtils
using Rotations, CoordinateTransformations
using PyLCM
using PyCall

using KernelDensityEstimate

using JLD, HDF5

lcmtpath = joinpath(dirname(@__FILE__),"lcmtypes")

# println("Compile LCM types")
# run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/rome_pose_node_t.lcm`)
# run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/rome_pose_pose_nh_t.lcm`)

println("Adding lcmtypes dir to Python path: $(lcmtpath)")
unshift!(PyVector(pyimport("sys")["path"]),lcmtpath)

@pyimport rome


function whatsthere(channel, msgdata)
  odomsg = rome.pose_pose_nh_t[:decode](msgdata)
  @show odomsg[:utime]
  nothing
end


vc = startdefaultvisualization()
sleep(3.0)
rovt = loadmodel(:rov)
rovt(vc)

# lastpose = SE3(0)

function lcmposenodes!(vc::VisualizationContainer, slaml::SLAMWrapper, msgdata)
  posenode = rome.pose_node_t[:decode](msgdata)
  # @show posenode[:id]
  tA = [posenode[:mean][1];posenode[:mean][2];posenode[:mean][3]]
  EA = Euler(posenode[:mean][6], posenode[:mean][5], posenode[:mean][4])
  q = TransformUtils.convert(Quaternion, EA)
  newpose = SE3(tA, q)
  am = Translation(tA...) ∘ LinearMap(Quat(q.s,q.v...))
  # updaterealtime!(vc,:body, am)
  # visualizerealtime(vc)

  # Dx = slaml.lastpose\newpose # get delta transform between poses
  # copy!(slaml.lastpose.t, newpose.t)
  # copy!(slaml.lastpose.R.R, newpose.R.R)

  if posenode[:id] == 0
    # first pose prior
    dcovar = [posenode[:covar][1];posenode[:covar][2];posenode[:covar][3];posenode[:covar][4];posenode[:covar][5];posenode[:covar][6]]
    slaml.fg = identitypose6fg(initpose=newpose, initCov=diagm(dcovar))
    @show lbls, = ls(slaml.fg)
    slaml.usrid2lbl[posenode[:id]] = lbls[1]
    slaml.lbl2usrid[lbls[1]] = posenode[:id]
  end


  rovt( vc, am )
  # change to using pose prior and interpose translation only
  # println("Added node $(v.label)")
  nothing
end

function lcmodomsg!(vc::VisualizationContainer, slaml::SLAMWrapper, msgdata)
  odomsg = rome.pose_pose_nh_t[:decode](msgdata)
  # @show odomsg[:mean_dim]
  # @show odomsg[:node_1_id], odomsg[:node_2_id]
  @show odomsg[:confidence]
  tA = [odomsg[:mean][1];odomsg[:mean][2];odomsg[:mean][3]]
  EA = Euler(odomsg[:mean][6], odomsg[:mean][5], odomsg[:mean][4])
  Dx = SE3(tA, EA)
  dcovar = [odomsg[:covar][1];odomsg[:covar][2];odomsg[:covar][3];odomsg[:covar][4];odomsg[:covar][5];odomsg[:covar][6]]

  if abs(odomsg[:confidence] - 1.0) < 1e-10
    # tA = [odomsg[:mean][1];odomsg[:mean][2];odomsg[:mean][3]]
    # EA = Euler(odomsg[:mean][6], odomsg[:mean][5], odomsg[:mean][4])
    # Dx = SE3(tA, EA)
    # dcovar = [odomsg[:covar][1];odomsg[:covar][2];odomsg[:covar][3];odomsg[:covar][4];odomsg[:covar][5];odomsg[:covar][6]]
    v,f = addOdoFG!(slaml, Pose3Pose3( Dx, diagm(dcovar) ) , saveusrid=odomsg[:node_2_id] )
    @show ls(slaml.fg)
  else
    # loop closure case
    println("LOOP CLOSURE")
    @show odomsg[:node_1_id], odomsg[:node_2_id]
    odoc3 = Pose3Pose3NH(Dx, diagm(dcovar), [0.5;0.5]) # define 50/50% hypothesis
    addFactor!(slam.fg,[Symbol("x$(odomsg[:node_1_id]+1)");Symbol("x$(odomsg[:node_2_id]+1)")],odoc3)
  end
  visualizeallposes!(vc, slaml.fg)

  nothing
end


function runlistener(
      fl::Vector{Bool},
      slam::SLAMWrapper,
      vc::VisualizationContainer;
      until::Symbol=:x999 ) #slam::SLAMWrapper)
  #
  gg = (channel, msg_data) -> lcmposenodes!(vc, slam, msg_data)

  odohdl = (channel, msg_data) -> lcmodomsg!(vc, slam, msg_data)

  print("Preparing LCM...")
  lc = LCM()
  subscribe(lc, "ROME_NODES", gg)
  subscribe(lc, "ROME_FACTORS", odohdl)
  while fl[1]
    handle(lc)
    if slam.lastposesym == until
      break
    end
  end
  println("done")
  nothing
end

function setupSLAMinDB()
  SLAMWrapper(initfg(sessionname="HAUV"), nothing, 0)
end

function lcmsendpose(lc, vert::Graphs.ExVertex, usrid::Int)
  msg = rome.pose_node_t()
  # p = getVertKDE(vert)
  p = getKDE(vert)
  mv = getKDEfit(p)
  # pme = getKDEMean(vert)
  msg[:utime] = 0
  msg[:id] = usrid
  msg[:mean_dim] = 6
  msg[:mean] = mv.μ
  msg[:covar_dim] = 6
  msg[:covar] = diag(mv.Σ)

  publish(lc, "MMISAM_POSE_EST_MEAN", msg)
  nothing
end
function lcmsendallposes(lc, slaml::SLAMWrapper)
  for lbluid in slaml.lbl2usrid
    @show lbluid
    lcmsendpose(lc, getVert(slaml.fg, lbluid[1]), lbluid[2])
  end
  nothing
end


# am = Translation(0.0,0,0) ∘ LinearMap(Quat(0.0,1.0,0,0))
# updaterealtime!(vc,:body, am)
# visualizerealtime(vc)
slam = setupSLAMinDB()

flags = Bool[1]
flags[1] = true

@async runlistener(flags, slam, vc, until=:x67)
println("LCM listener running...")



# run this to stop the lcm listener loop
flags[1] = false


solveandvisualize(slam.fg, vc, drawtype=:fit) #, densitymeshes=[:x1;:x33;:x60])

visualizeallposes!(vc, slam.fg, drawtype=:max)




lc = LCM()
lcmsendallposes(lc, slam)



visualizeDensityMesh!(vc, slam.fg, :x1,meshid=2)


# savefg = deepcopy(slam.fg)
# savefg.registeredModuleFunctions = nothing
# @save joinpath(dirname(@__FILE__),"hauvnh67.jld") savefg






using Gadfly


draw(PDF("/home/dehann/Desktop/test.pdf",30cm,20cm),
        plotKDE( marginal(p60,collect(4:6)),
				dimLbls=["x";"y";"z";"phi";"the";"psi"]) )








#
