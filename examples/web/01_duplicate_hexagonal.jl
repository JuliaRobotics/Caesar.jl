# duplication hex example
# add more julia processes
using Distributed
nprocs() < 3 ? addprocs(4-nprocs()) : nothing

using Caesar
using GraffSDK
## Inter-operating visualization packages for Caesar/RoME/IncrementalInference exist
using RoMEPlotting


# Graff.....
# 1. Import the initialization code.
cd(joinpath(dirname(pathof(GraffSDK)), "..", "examples"))
# 1a. Create a Configuration
config = loadGraffConfig("synchronyConfig1.json")
#Create a hexagonal sessions
config.sessionId = "HexDemo01"
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

## LOCAL MEMORY
# start with an empty factor graph object
fg = initfg()
fg.isfixedlag = true
fg.qfl = 10

# Add the first pose :x0
addNode!(fg, :x0, Pose2)

# Add at a fixed location PriorPose2 to pin :x0 to a starting location (0, 0, 0)
addFactor!(fg, [:x0], IIF.Prior( MvNormal([0; 0; 0], Matrix(Diagonal([0.1;0.1;0.05].^2)) )))

## CLOUD VERSION
# This code should work for the first session created, but we need to do a little work so the code is good for infinitely many new sessions
# TODO: manange non-(0,0,0) initialization of new sessions -- i.e. using landmark initialization.
if !isSessionExisting()
    # this call in current API will also create the initial x0 pose with a prior (0,0,0)
    @info "Session $(config.sessionId) doesn't exist, creating.."
    newSessionRequest = SessionDetailsRequest(config.sessionId, "A test dataset demonstrating data ingestion for a wheeled vehicle driving in a hexagon.", "Pose2")
    session = addSession(newSessionRequest)
else
    @info "Session $(config.sessionId) already exists, carry on!"
end


# Drive around in a hexagon
for i in 0:5
  # define previous and next pose symbols
  psym = Symbol("x$i")
  nsym = Symbol("x$(i+1)")
  deltaMeasurement = [10.0;0;pi/3]
  pOdo = Float64[0.1 0 0; 0 0.1 0; 0 0 0.1]
  pp = Pose2Pose2(MvNormal(deltaMeasurement, pOdo.^2))

  ## LOCAL MEMORY
  # add next pose variable to local memory version
  addNode!(fg, nsym, Pose2)
  # add odometry factor to local memory version
  addFactor!(fg, [psym;nsym], pp )

  ## CLOUD MEMORY
  # duplicate next pose variable on cloud version
  # TODO: addVariable!(fg_remote, nsym, Pose2)
  # duplicate odometry factor on cloud version
  # TODO addFactor!(fg_remote, [psym;nsym], pp )
  ## Existing addOdo interface
  newOdometryMeasurement = AddOdometryRequest(deltaMeasurement, pOdo)
  @time @show addOdoResponse = addOdometryMeasurement(newOdometryMeasurement)
end

# perform inference on local version, and remember first runs are slower owing to Julia's just-in-time compiling
batchSolve!(fg)

# No need to call batchSolve on cloud - it picks up on changes and
# 8. Let's check on the solver updates.
# sessionLatest = getSession()
# while session.lastSolvedTimestamp != sessionLatest.lastSolvedTimestamp
#   println("Comparing latest session solver timestamp $(sessionLatest.lastSolvedTimestamp) with original $(session.lastSolvedTimestamp) - still the same so sleeping for 2 seconds")
#   sleep(2)
#   sessionLatest = getSession()
# end
# get cloud nodes
getNodes()

# For Juno/Jupyter style use
pl = drawPoses(fg)
# For scripting use-cases you can export the image
Gadfly.draw(Gadfly.PDF("/tmp/test1.pdf", 20cm, 10cm),pl)  # or PNG(...)

# Add landmarks with Bearing range measurements
addNode!(fg, :l1, Point2, labels=["LANDMARK"])
p2br = Pose2Point2BearingRange(Normal(0,0.1), Normal(20.0,1.0))
addFactor!(fg, [:x0; :l1], p2br)

# Initialize :l1 numerical values but do not rerun solver
ensureAllInitialized!(fg)
pl = drawPosesLandms(fg)
Gadfly.draw(Gadfly.PDF("/tmp/test2.pdf", 20cm, 10cm),pl)  # or PNG(...)


# Add landmarks with Bearing range measurements
p2br2 = Pose2Point2BearingRange(Normal(0,0.1), Normal(20.0,1.0))
addFactor!(fg, [:x6; :l1], p2br2)

# solve
batchSolve!(fg)

# redraw
pl = drawPosesLandms(fg)
Gadfly.draw(Gadfly.PDF("/tmp/test3.pdf", 20cm, 10cm),pl)  # or PNG(...)










#
