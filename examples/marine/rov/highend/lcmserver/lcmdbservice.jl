# addprocs(4)

using Caesar, RoME
using TransformUtils
using Rotations, CoordinateTransformations
using PyLCM
using PyCall
using Distributions

using CloudGraphs

using KernelDensityEstimate

using JLD, HDF5

@show lcmtpath = joinpath(dirname(@__FILE__),"lcmtypes")

# println("Compile LCM types")
run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/rome_pose_node_t.lcm`)
run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/rome_pose_pose_nh_t.lcm`)
# run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/rome_point_cloud_t.lcm`)
run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/hauv_submap_points_t.lcm`)

println("Adding lcmtypes dir to Python path: $(lcmtpath)")
unshift!(PyVector(pyimport("sys")["path"]),lcmtpath)

@pyimport rome
@pyimport hauv

# random test and viewing function
# function whatsthere(channel, msgdata)
#   odomsg = rome.pose_pose_nh_t[:decode](msgdata)
#   @show odomsg[:utime]
#   nothing
# end

# ! = non-const function (eyecandy only)
# . = dot-notation (element-wise execution)
function lcmposenodes!(vc, slaml::SLAMWrapper, msgdata,
        lastposeRPZ::Vector{Float64}, MSGDATA;
        record=false,
        model=nothing )
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
    # initialize fg by setting the first pose w/ prior
    slaml.fg = identitypose6fg(fg=slaml.fg, initpose=newpose, initCov=diagm(dcovar))

# ???
    lbls, = ls(slaml.fg)
    slaml.usrid2lbl[posenode[:id]] = lbls[1]
    slaml.lbl2usrid[lbls[1]] = posenode[:id]
  end


  # update rov pose in visuzalizer
  # TODO: add rovt as argument to fcn
  model( vc, am )

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
  dcovar = dcovar

  # println("lcmodomsg! mid")

  if abs(odomsg[:confidence] - 1.0) < 1e-10
    constrs = IncrementalInference.FunctorInferenceType[] # factor super type
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

    xyy = Pose3Pose3( MvNormal([tA; EA], diagm(dcovar)^2 ) )

    push!( constrs, prpz )
    push!( constrs, xyy )

    # may or may not add a new pose node, depending on message order
    v,f = addposeFG!(slaml, constrs, saveusrid=odomsg[:node_2_id], ready=0 )
    setDBAllReady!(slaml.fg)

  else
    # loop closure case
    println("LOOP CLOSURE")
    odoc3 = Pose3Pose3NH( MvNormal(veeEuler(Dx), diagm(1.0./dcovar)), [0.5;0.5]) # define 50/50% hypothesis
    between_verts = Symbol[Symbol("x$(odomsg[:node_1_id]+1)");Symbol("x$(odomsg[:node_2_id]+1)")]
    # order in between_verts must match the order assumed in the factor functor
    addFactor!(slaml.fg,between_verts,odoc3)
  end

  # update viewer
  visualizeallposes!(vc, slaml.fg)

  # println("lcmodomsg! end")
  nothing
end


# call back for point cloud messages
function lcmpointcloudmsg!(slaml::SLAMWrapper,
                          msgdata,
                          MSGDATA;
                          record::Bool=false  )
  #
  !record ? nothing : push!(MSGDATA[:submap_points_t], deepcopy(msgdata))

  msg = hauv.submap_points_t[:decode](msgdata)
  @show msg[:id]
  @show typeof(msg[:points_local])

  node_name =  Symbol("x$(odomsg[:node_1_id]+1)")
  # function appendvertbigdata!(
  vert = getVert(slaml.fg, )
  appendvertbigdata!(slaml.fg, )

  nothing
end

function runlistener!(
      fl::Vector{Bool},
      slam::SLAMWrapper,
      vc;
      MSGDATA=nothing,
      until::Symbol=:x999,
      record=false,
      model = nothing  )
  #
  lastposeRPZ = zeros(3)

  if record
    MSGDATA[:pose_node_t] = Vector{Vector{UInt8}}()
    MSGDATA[:pose_pose_nh_t] = Vector{Vector{UInt8}}()
    MSGDATA[:submap_points_t] = Vector{Vector{UInt8}}()
  end

  # closure
  gg = (channel, msg_data) -> lcmposenodes!(vc, slam, msg_data, lastposeRPZ, MSGDATA, record=record, model=model )
  odohdl = (channel, msg_data) -> lcmodomsg!(vc, slam, msg_data, lastposeRPZ, MSGDATA, record=record )
  pchdl = (channel, msg_data) -> lcmpointcloudmsg!(slam, msg_data, MSGDATA, record=record )


  print("Preparing LCM...")
  lc = LCM()
  # subscribe(lc, "ROME_NODES", gg)
  # subscribe(lc, "ROME_FACTORS", odohdl)
  # subscribe(lc, "ROME_POINTCLOUD", pchdl)
  subscribe(lc, "SUBMAPS", pchdl)

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

function setupSLAMinDB(;cloudGraph=nothing, addrdict=nothing)
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

# for local dictionary usage
# slam = setupSLAMinDB()

0

# for remote db usage
include(joinpath(dirname(@__FILE__),"..","database","blandauthremote.jl"))
addrdict["session"] = "SESSHAUVDEV"
# addrdict are the server credentials and session id
# if none are given, will prompt user
cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)
# # cloudGraph, addrdict = standardcloudgraphsetup()
slam = setupSLAMinDB(cloudGraph=cloudGraph, addrdict=addrdict)


flags = Bool[true]

MSGDATA = Dict{Symbol, Vector{Vector{UInt8}}}()


println("LCM listener running...")
# can call @async runlistener!)
@async runlistener!(flags, slam, vc, until=:x1, MSGDATA=MSGDATA, record=false, model=rovt)
flags[1] = false

####

MSGDATA[:submap_points_t]
msg = hauv.submap_points_t[:decode](MSGDATA[:submap_points_t][1])

hauv.submap_points_t




# slamdb10 = deepcopy(slam)

savejld(slam.fg, file="hauvpedro.jld")


fullLocalGraphCopy!(slam.fg)


writeGraphPdf(slam.fg)
run(`evince fg.pdf`)

#

# # visualizeDensityMesh!(vc, slam.fg, :x30)
visualizeallposes!(vc, slam.fg, drawtype=:fit)

f1 = getfnctype(slam.fg, slam.fg.fIDs[:x35x57])
f1.Zij






# loopclosures
@show lcfncs = lsf(slam.fg, Pose3Pose3NH)



session = addrdict["session"]
@show IDs = getPoseExVertexNeoIDs(cloudGraph.neo4j.connection, session=session, reqbackendset=false);



using IncrementalInference


plotKDEresiduals(slam.fg, :x15x61, marg=[1;2], N=Nparticles)



using Gadfly

plotMCMC(tree, :x60, dims=[1;2], levels=1)



tree = wipeBuildNewTree!(slam.fg, drawpdf=true)
@async run(`evince bt.pdf`)


inferOverTree!(slam.fg,tree,N=Nparticles, dbg=true)

pl = spyCliqMat(tree, :x61)
Gadfly.draw(PDF("spycliqmatx61.pdf",15cm,20cm),pl)
run(`evince spycliqmatx61.pdf`)
run(`cp hauv67.jld /home/dehann/Pictures/hauv/`)


[solve(slam.fg) for i in 1:1] #, densitymeshes=[:x1;:x33;:x60])
visualize(slam.fg, vc, drawtype=:fit)

using IncrementalInference

fg = slam.fg

fg = slamdict10.fg

getVal(fg, :x1)

fullLocalGraphCopy!(slam.fg)

sp = plotKDEresiduals(fg, :x1x2, marg=[1;2], N=Nparticles)
sp = plotKDEresiduals(fg, :x1x2, marg=[3;6], N=Nparticles)

# Gadfly.draw(PNG("/home/dehann/Pictures/sp3.png",20cm,15cm),sp)



savejld(fg) # file=tempfg.jld


using RoME

# Juno.breakpoint("/home/dehann/.julia/v0.5/RoME/src/fgos.jl",19)

fgu = loadjld(file="test.jld")

writeGraphPdf(fgu)
run(`evince fg.pdf`)

# end


1
# # loopclosures
# @show lcfncs = lsf(fg, Pose3Pose3NH)
# #

#
#
# # plotKDEofnc(fg, :x1x2, marg=[1;2], N=Nparticles)
#
# donum = 5
# plotKDEofnc(fg, lcfncs[donum], marg=[1;2], N=Nparticles)
# plotKDEofnc(fg, lcfncs[donum], marg=[6;3], N=Nparticles)
# plotKDEofnc(fg, lcfncs[donum], marg=[4;5], N=Nparticles)
#
#


fgu = loadjld(file="hauvpedro.jld")

plotKDE(fgu, :x60, marg=[1;2])
#

IncrementalInference.plotKDEresiduals(fgu, :x20x21, marg=[1;2], N=100)

IncrementalInference.lsf(fgu, PartialPriorRollPitchZ)




# #
# #
# # println("AD-HOC DEBUGGING")
# #
# # slam = setupSLAMinDB()
# #
# #
# # lcmposenodes!(vc, slam, MSGDATA[:pose_node_t][1], record=false)
# # lcmposenodes!(vc, slam, MSGDATA[:pose_node_t][2], record=false)
# #
# #
# # lcmodomsg!(vc, slam, MSGDATA[:pose_pose_nh_t][1], record=false)
# #
















#
