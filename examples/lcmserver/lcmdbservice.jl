addprocs(4)

using Caesar, RoME
using TransformUtils
using Rotations, CoordinateTransformations
using PyLCM
using PyCall
using Distributions

using KernelDensityEstimate

using JLD, HDF5

@show lcmtpath = joinpath(dirname(@__FILE__),"lcmtypes")

# println("Compile LCM types")
# run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/rome_pose_node_t.lcm`)
# run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/rome_pose_pose_nh_t.lcm`)

println("Adding lcmtypes dir to Python path: $(lcmtpath)")
unshift!(PyVector(pyimport("sys")["path"]),lcmtpath)

@pyimport rome

# random test and viewing function
# function whatsthere(channel, msgdata)
#   odomsg = rome.pose_pose_nh_t[:decode](msgdata)
#   @show odomsg[:utime]
#   nothing
# end

function lcmposenodes!(vc, slaml::SLAMWrapper, msgdata,
        lastposeRPZ::Vector{Float64}, MSGDATA;
        record=false  )
  #
  println("lcmposenodes! start")
  !record ? nothing : push!(MSGDATA[:pose_node_t], deepcopy(msgdata))

  posenode = rome.pose_node_t[:decode](msgdata)
  @show posenode[:id]
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
  # println("lcmposenodes! mid")

  lastposeRPZ[1], lastposeRPZ[2] = posenode[:mean][6], posenode[:mean][5]
  lastposeRPZ[3] = posenode[:mean][3]

  if posenode[:id] == 0
    # first pose prior
    dcovar = [posenode[:covar][1];posenode[:covar][2];posenode[:covar][3];posenode[:covar][4];posenode[:covar][5];posenode[:covar][6]]
    # TODO -- this wont do, must add to existing slaml.fg
    slaml.fg = identitypose6fg(fg=slaml.fg, initpose=newpose, initCov=diagm(dcovar))

    lbls, = ls(slaml.fg)
    slaml.usrid2lbl[posenode[:id]] = lbls[1]
    slaml.lbl2usrid[lbls[1]] = posenode[:id]
  end


  rovt( vc, am )
  # change to using pose prior and interpose translation only
  # println("Added node $(v.label)")
  # println("lcmposenodes! end")
  nothing
end


function lcmodomsg!(vc, slaml::SLAMWrapper, msgdata,
        lastposeRPZ::Vector{Float64}, MSGDATA;
        record=false  )
  #
  println("lcmodomsg! start")
  @show lastposeRPZ
  !record ? nothing : push!(MSGDATA[:pose_pose_nh_t], deepcopy(msgdata))

  odomsg = rome.pose_pose_nh_t[:decode](msgdata)
  # @show odomsg[:mean_dim]
  @show odomsg[:node_1_id], odomsg[:node_2_id]
  # @show odomsg[:confidence]
  @show tA = [odomsg[:mean][1];odomsg[:mean][2];odomsg[:mean][3]]
  roll,pitch,yaw = odomsg[:mean][6], odomsg[:mean][5], odomsg[:mean][4]
  println("dXYZ=$(round(tA, 2)), dRPY=$(round(collect((roll,pitch,yaw)),3))")
  EA = Euler(roll,pitch,yaw)
  Dx = SE3(tA, EA)
  dcovar = [odomsg[:covar][1];odomsg[:covar][2];odomsg[:covar][3];odomsg[:covar][6];odomsg[:covar][5];odomsg[:covar][4]]

  # println("lcmodomsg! mid")

  if abs(odomsg[:confidence] - 1.0) < 1e-10
    constrs = IncrementalInference.FunctorInferenceType[]
    println("adding PartialPriorRollPitchZ")
    prpz = PartialPriorRollPitchZ(
      MvNormal(  [lastposeRPZ[1];lastposeRPZ[2]],diagm([odomsg[:covar][6];odomsg[:covar][5]])  ),
      Normal(  lastposeRPZ[3],odomsg[:covar][3]  )
    )
    # println("adding PartialPose3XYYaw")
    # xyy = PartialPose3XYYaw(
    #   MvNormal([odomsg[:mean][1];odomsg[:mean][2];odomsg[:mean][4]],
    #            [odomsg[:covar][1];odomsg[:covar][2];odomsg[:covar][4]] )
    # )
    xyy = Pose3Pose3( Dx, diagm(dcovar) )

    push!( constrs, prpz )
    push!( constrs, xyy )

    v,f = addposeFG!(slaml, constrs, saveusrid=odomsg[:node_2_id] )
  else
    # loop closure case
    println("LOOP CLOSURE")
    odoc3 = Pose3Pose3NH(Dx, diagm(1.0./dcovar), [0.5;0.5]) # define 50/50% hypothesis
    addFactor!(slam.fg,[Symbol("x$(odomsg[:node_1_id]+1)");Symbol("x$(odomsg[:node_2_id]+1)")],odoc3)
  end
  visualizeallposes!(vc, slaml.fg)

  # println("lcmodomsg! end")
  nothing
end



function runlistener!(
      fl::Vector{Bool},
      slam::SLAMWrapper,
      vc;
      MSGDATA=nothing,
      until::Symbol=:x999,
      record=false  )
  #
  lastposeRPZ = zeros(3)

  if record
    MSGDATA[:pose_node_t] = Vector{Vector{UInt8}}()
    MSGDATA[:pose_pose_nh_t] = Vector{Vector{UInt8}}()
  end

  gg = (channel, msg_data) -> lcmposenodes!(vc, slam, msg_data, lastposeRPZ, MSGDATA, record=record )
  odohdl = (channel, msg_data) -> lcmodomsg!(vc, slam, msg_data, lastposeRPZ, MSGDATA, record=record )

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

function setupSLAMinDB(cloudGraph=nothing, addrdict=nothing)
  if cloudGraph != nothing
    return SLAMWrapper(Caesar.initfg(sessionname=addrdict["session"], cloudgraph=cloudGraph), nothing, 0)
  else
    return SLAMWrapper(RoME.initfg(), nothing, 0)
  end
end


vc = startdefaultvisualization()
sleep(3.0)
rovt = loadmodel(:rov)
rovt(vc)


Nparticles = 100
# include(joinpath(dirname(@__FILE__),"..","database","blandauthremote.jl"))
# addrdict["session"] = "SESSHAUV"
# cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)
# # cloudGraph, addrdict = standardcloudgraphsetup()
# slam = setupSLAMinDB(cloudGraph, addrdict)

slam = setupSLAMinDB()


flags = Bool[1]
flags[1] = true

MSGDATA = Dict{Symbol, Vector{Vector{UInt8}}}()

println("LCM listener running...")
runlistener!(flags, slam, vc, until=:x10, MSGDATA=MSGDATA, record=false)



# visualizeDensityMesh!(vc, slam.fg, :x30)
visualizeallposes!(vc, slam.fg, drawtype=:fit)

[solveandvisualize(slam.fg, vc, drawtype=:fit) for i in 1:1] #, densitymeshes=[:x1;:x33;:x60])



println("Debugging functions")
using IncrementalInference

fg = slam.fg

# loopclosures
@show lcfncs = lsf(fg, Pose3Pose3NH)




# plotKDEofnc(fg, :x1x2, marg=[1;2], N=Nparticles)

donum = 5
plotKDEofnc(fg, lcfncs[donum], marg=[1;2], N=Nparticles)
plotKDEofnc(fg, lcfncs[donum], marg=[6;3], N=Nparticles)
plotKDEofnc(fg, lcfncs[donum], marg=[4;5], N=Nparticles)



plotKDEresiduals(fg, :x1x2, marg=[6;3], N=Nparticles)


fnc = getfnctype(fg, fg.fIDs[:x33x34])
fnc.Zij

pts = getSample(fnc, Nparticles)[1]
Base.mean(pts,2)

ls(fg, :x55)

msgdata = MSGDATA[:pose_node_t][3]
odomsg = rome.pose_node_t[:decode](msgdata)
@show odomsg[:id]
@show odomsg[:mean]


ft = getVert(fg, fg.fIDs[:x2])
fieldnames(getData(ft).fnc.usrfnc!)
getData(ft).fnc.usrfnc!.rp





getData(ft).fnc.usrfnc!.z




meas = getSample(getData(ft).fnc.usrfnc!)




p = findRelatedFromPotential(fg, ft, fg.IDs[:x2], Nparticles)
plotKDE(marginal(p,[3]))


val = predictbelief(fg, :x2, [:x2], N=Nparticles)






ppx2 = marginal(kde!(val),[3])
plotKDE(ppx2, levels=3)




plotKDE([px1;ppx2],c=["red";"green"], levels=4)



#
#
ls(slam.fg)
writeGraphPdf(slam.fg)
run(`evince fg.pdf`)
#
#
# println("AD-HOC DEBUGGING")
#
# slam = setupSLAMinDB()
#
#
# lcmposenodes!(vc, slam, MSGDATA[:pose_node_t][1], record=false)
# lcmposenodes!(vc, slam, MSGDATA[:pose_node_t][2], record=false)
#
#
# lcmodomsg!(vc, slam, MSGDATA[:pose_pose_nh_t][1], record=false)
#
















#
