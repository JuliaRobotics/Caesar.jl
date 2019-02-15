# duplication hex example
# add more julia processes
using Distributed
nprocs() < 3 ? addprocs(4-nprocs()) : nothing

using Caesar
using GraffSDK
using UUIDs
## Inter-operating visualization packages for Caesar/RoME/IncrementalInference exist
# using RoMEPlotting


# Graff.....
# 1. Import the initialization code.
cd(joinpath(dirname(pathof(GraffSDK)), "..", "examples"))
# 1a. Create a Configuration
config = loadGraffConfig("synchronyConfig1.json")
#Create a hexagonal sessions
config.sessionId = "Hexagonal_"*replace(string(uuid4()), "-" => "")
println(getGraffConfig())
# 1b. Check the credentials and the service status
printStatus()

# Make sure that the session and robot exist.
if !isRobotExisting()
    @info "Robot $(config.robotId) doesn't exist, creating.."
    newRobot = RobotRequest(config.robotId, "My New Bot", "Description of my neat robot", "Active");
    robot = addRobot(newRobot);
else
    @info "Robot $(config.robotId) already exists, carry on!"
end
# ----- Done!

## Fetch existing marginals from previous sessions
# TODO: new GraffSDK call to fetch desired landmark marginals
existingSessions = getSessions()
# TODO: Get LANDMARK MAP_Ests
# use these LANDMARK estimates as priors in the local version when the same tags seen again
# getNodes()
# Decide which ones we want...

# Make a new session
session = nothing
if !isSessionExisting()
    println(" -- Session '$(config.sessionId)' doesn't exist for robot '$(config.robotId)', creating it...")
    newSessionRequest = SessionDetailsRequest(config.sessionId, "Hexagonal duplication example.", "Pose2", false) #Don't initialize it.
    session = addSession(newSessionRequest)
else
    session = getSession()
end

## LOCAL MEMORY
# start with an empty factor graph object
fg = initfg()
fg.isfixedlag = true
fg.qfl = 10

# Local: Add the first pose :x0
addVariable!(fg, :x0, Pose2)
# Graff: Add x0 to Graff
addVariable(:x0, Pose2, String[])

# Add at a fixed location PriorPose2 to pin :x0 to a starting location (0, 0, 0)
prior = IIF.Prior( MvNormal([0; 0; 0], Matrix(Diagonal([0.1;0.1;0.05].^2)) ))
# Local: Add a prior to x0
addFactor!(fg, [:x0], prior)
# Graff: Add the prior to x0
addFactor([:x0], prior)

# Drive around in a hexagon
for i in 0:5
  # define previous and next pose symbols
  psym = Symbol("x$i")
  nsym = Symbol("x$(i+1)")
  deltaMeasurement = [10.0;0;pi/3]
  pOdo = Float64[0.1 0 0; 0 0.1 0; 0 0 0.1]
  pp = Pose2Pose2(MvNormal(deltaMeasurement, pOdo.^2))

  ## LOCAL MEMORY
  # Local: add next pose variable +odometry to local memory version
  addVariable!(fg, nsym, Pose2)
  addFactor!(fg, [psym;nsym], pp )
  # Graff: add odometry
  addVariable(nsym, Pose2, String[])
  addFactor([psym;nsym], pp)
end

# Graff: Let the cloud instance know that the graph is ready
putReady(true)

# perform inference on local version, and remember first runs are slower owing to Julia's just-in-time compiling
batchSolve!(fg)

# No need to call batchSolve on cloud - it picks up on changes and
# 8. Let's check on the solver updates.
sessionLatest = getSession()
while session.lastSolvedTimestamp != sessionLatest.lastSolvedTimestamp
  println("Comparing latest session solver timestamp $(sessionLatest.lastSolvedTimestamp) with original $(session.lastSolvedTimestamp) - still the same so sleeping for 2 seconds")
  sleep(2)
  sessionLatest = getSession()
end
# get cloud nodes
getNodes()

# For Juno/Jupyter style use
pl = drawPoses(fg)
# For scripting use-cases you can export the image
Gadfly.draw(Gadfly.PDF("/tmp/test1.pdf", 20cm, 10cm),pl)  # or PNG(...)

# Add landmarks with Bearing range measurements
addVariable!(fg, :l1, Point2, labels=["LANDMARK"])
addVariable(:l1, Point2, ["LANDMARK"])
p2br = Pose2Point2BearingRange(Normal(0,0.1), Normal(20.0,1.0))
addFactor!(fg, [:x0; :l1], p2br)
addFactor([:x0, :l1], p2br)

# Initialize :l1 numerical values but do not rerun solver
ensureAllInitialized!(fg)
pl = drawPosesLandms(fg)
Gadfly.draw(Gadfly.PDF("/tmp/test2.pdf", 20cm, 10cm),pl)  # or PNG(...)

# Add landmarks with Bearing range measurements
p2br2 = Pose2Point2BearingRange(Normal(0,0.1), Normal(20.0,1.0))
addFactor!(fg, [:x6; :l1], p2br2)
addFactor([:x6, :l1], p2br2)

# Graff: PutReady to tell Graff to solve
putReady(true)
# Local: Solve
batchSolve!(fg)

# If you like you can request a full graph solve
requestSessionSolve()

# redraw
pl = drawPosesLandms(fg)
Gadfly.draw(Gadfly.PDF("/tmp/test3.pdf", 20cm, 10cm),pl)  # or PNG(...)










#
