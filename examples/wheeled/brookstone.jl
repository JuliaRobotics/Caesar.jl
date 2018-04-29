### Some playtime with the little Rover2.
### REF: http://blog.leahhanson.us/post/julia/julia-calling-python.html

# Installs
# Pkg.add("Images")
# Pkg.add("ImageView")
# Pkg.add("PyCall")
# Pkg.add("CloudGraphs")
# Pkg.add("ProgressMeter")
# Pkg.add("JSON")
# Pkg.add("Unmarshal")

using Base
using PyCall
using FileIO
using SynchronySDK
using ArgParse
# include("ExtensionMethods.jl")

@everywhere include("./entities/RoverPose.jl")
@everywhere include("./entities/SystemConfig.jl")

# Allow the local directory to be used
cd(Pkg.dir("RoverPilot"))
unshift!(PyVector(pyimport("sys")["path"]), "")

# Globals
shouldRun = true
# Let's use some local FIFO's here.
@everywhere sendPoseQueue = Channel{RoverPose}(100)

### SYNCHRONY CONFIG
using Base
using JSON, Unmarshal
using SynchronySDK
using SynchronySDK.DataHelpers

# 0. Constants
curtime = now()
robotId = "Broekiestone"
sessionId = "RoverRover"*"_$(Dates.year(curtime))$(Dates.month(curtime))$(Dates.day(curtime))T$(Dates.hour(curtime))_$(Dates.minute(curtime))_$(Dates.second(curtime))"

# 1. Get a Synchrony configuration
println(" - Retrieving Synchrony Configuration...")
configFile = open(joinpath("config", "synchronyConfig_Local.json"))
configData = JSON.parse(readstring(configFile))
close(configFile)
synchronyConfig = Unmarshal.unmarshal(SynchronyConfig, configData)

println(" --- Configuring Synchrony example for:")
println("  --- User: $(synchronyConfig.userId)")
println("  --- Robot: $(robotId)")
println("  --- Session: $(sessionId)")

# 2. Confirm that the robot already exists, create if it doesn't.
println(" - Creating or retrieving robot '$robotId'...")
robot = nothing
if(SynchronySDK.isRobotExisting(synchronyConfig, robotId))
    println(" -- Robot '$robotId' already exists, retrieving it...")
    robot = getRobot(synchronyConfig, robotId)
else
    # Create a new one
    println(" -- Robot '$robotId' doesn't exist, creating it...")
    newRobot = RobotRequest(robotId, "Brookstone Rover", "Weekend hackathon time!", "Active")
    robot = addRobot(synchronyConfig, newRobot)
end
println(robot)

# 3. Create or retrieve the session.
# Get sessions, if it already exists, add to it.
println(" - Creating or retrieving data session '$sessionId' for robot...")
session = nothing
if(SynchronySDK.isSessionExisting(synchronyConfig, robotId, sessionId))
    println(" -- Session '$sessionId' already exists for robot '$robotId', retrieving it...")
    session = getSession(synchronyConfig, robotId, sessionId)
else
    # Create a new one
    println(" -- Session '$sessionId' doesn't exist for robot '$robotId', creating it...")
    newSessionRequest = SessionDetailsRequest(sessionId, "A test dataset demonstrating data ingestion for a wheeled vehicle driving in a hexagon.", "Pose2")
    session = addSession(synchronyConfig, robotId, newSessionRequest)
end
println(session)
######

"""
Send odometry and camera data to database at each keyframe/pose instantiation event via SynchronySDK.
"""
function nodeTransmissionLoop(sysConfig::SystemConfig, robotId::String, sessionId::String, synchronyConfig::SynchronyConfig)
    # Get from sysConfig
    # Podo=diagm([0.1;0.1;0.005]) # TODO ASK: Noise?
    N=100

    println("[SendNodes Loop] Started!")
    while shouldRun
        # try
        @show pose = take!(sendPoseQueue)
        println("[SendNodes Loop] SendQueue got message, sending $(poseIndex(pose))!")

        @show deltaMeasurement = odoDiff(pose) #[10.0;0;pi/3]
        pOdo = diagm([0.1;0.1;0.005])
        println(" - Measurement: Adding new odometry measurement '$deltaMeasurement'...")
        newOdometryMeasurement = AddOdometryRequest(deltaMeasurement, pOdo)
        addOdoResponse = addOdometryMeasurement(synchronyConfig, robotId, sessionId, newOdometryMeasurement)
        println(" - Adding image data to node with label $(addOdoResponse.variable.label)")
        index = 0
        for robotImg = pose.camImages
            bigDataImage = BigDataElementRequest("Camera_$index", "Mongo", description, base64encode(robotImg.camJpeg), "image/jpeg")
            index = index + 1
        end

        #
        #     @time lastPoseVertex, factorPose = addOdoFG!(fg, poseIndex(pose), odoDiff(pose), Podo, N=N, labels=["VARIABLE", pose.poseId, sysConfig.botId])
        #     # Now send the images.
        #     for robotImg = pose.camImages
        #         ksi = ImageData(string(Base.Random.uuid4()), pose.poseId, sysConfig.sessionId, String(poseIndex(pose)), robotImg.timestamp, robotImg.camJpeg, "jpg", Dict{String, String}())
        #         @time sendRawImage(kafkaService, ksi)
        #     end
        # catch e
        #     println("[SendNodes Loop] Error seding node!")
        #     bt = catch_backtrace()
        #     showerror(STDOUT, e, bt)
        # end
        println("[SendNodes Loop] Sent message!")
        # sleep(1)
    end
    println("[SendNodes Loop] Done!")
end

function juliaDataLoop(sysConfig::SystemConfig, rover)
    # Make the initial pose, assuming start pose is 0,0,0 - setting the time to now.
    curPose = RoverPose()
    curPose.timestamp = Base.Dates.datetime2unix(now())
    # Bump it to x2 because we already have x1 (curPose = proposed pose, not saved yet)
    curPose.poseIndex = 2

    lastSend = Nullable{Task}

    println("[Julia Data Loop] Should run = $shouldRun");
    while shouldRun
        # Update the data acquisition
        rover[:iterateDataProcessor]()
        # Check length of queue
        # println("[Julia Data Loop] Image frame count = $frameCount");
        while rover[:getRoverStateCount]() > 0
            roverState = rover[:getRoverState]()
            append!(curPose, roverState, sysConfig.botConfig.maxImagesPerPose)
#            println(curPose)
            if (isPoseWorthy(curPose))
                println("Promoting Pose $(poseIndex(curPose)) to CloudGraphs!")
                put!(sendPoseQueue, curPose)
                curPose = RoverPose(curPose) # Increment pose
            end
            sleep(0.005)
        end
    end
    print("[Julia Data Loop] I'm out!");
end

"""
Get the command-line parameters.
"""
function parse_commandline()
    s = ArgParseSettings()

    @add_arg_table s begin
        "sysConfig"
            help = "Provide a system configuration file"
            default = "./config/systemconfig_aws.json"
    end

    return parse_args(s)
end

function main()
    # Parse command lines.
    parsedArgs = parse_commandline()

    println(" --- Loading system config from '$(parsedArgs["sysConfig"])'...")
    sysConfig = readSystemConfigFile(parsedArgs["sysConfig"])
    sysConfig.sessionId = sysConfig.botId * "_" * (!isempty(strip(sysConfig.sessionPrefix)) ? sysConfig.sessionPrefix * "_" : "") * string(Base.Random.uuid1())[1:8] #Name+SHA

    # Connect to CloudGraphs
    # println(" --- Connecting to CloudGraphs instance $(sysConfig.cloudGraphsConfig.neo4jHost)...")
    # cloudGraph = connect(sysConfig.cloudGraphsConfig);
    # conn = cloudGraph.neo4j.connection;
    # # register types of interest in CloudGraphs
    # registerGeneralVariableTypes!(cloudGraph)
    # Caesar.usecloudgraphsdatalayer!()
    # println("Current session: $(sysConfig.sessionId)")

    # Kafka Initialization - callbacks not used yet.
    # Kafka Initialization
    # println(" --- Connecting to Kafka instance $(sysConfig.kafkaConfig.ip)...")
    # function sessionMessageCallback(message)
    #     @show typeof(message)
    # end
    # function robotMessageCallback(message)
    #     @show message
    # end
    # kafkaService = KafkaService(sysConfig)
    # initialize(kafkaService, Vector{KafkaConsumer}())
    # # Send a status message to say we're up!
    # sendStatusNotification(kafkaService, StatusNotification(sysConfig.botId, sysConfig.sessionId, "ACTIVE"))

    # # Now start up our factor graph.
    # fg = Caesar.initfg(sessionname=sysConfig.sessionId, cloudgraph=cloudGraph)
    # Initialize the factor graph and insert first pose.
    # lastPoseVertex = initFactorGraph!(fg, labels=[sysConfig.botId])

    # println(" --- Starting out transmission loop!")
    sendLoop = @async nodeTransmissionLoop(sysConfig, robotId, sessionId, synchronyConfig)

    # Let's do some importing
    # Ref: https://github.com/JuliaPy/PyCall.jl/issues/53
    println(" --- Connecting to Rover!")
    roverModule = pyimport("RoverPylot")
    rover = roverModule[:PS3Rover](sysConfig.botConfig.deadZoneNorm, sysConfig.botConfig.maxRotSpeed, sysConfig.botConfig.maxTransSpeed)
    # Initialize
    rover[:initialize]()

    println(" --- Current session: $(sessionId)")

    # Start the main loop
    println(" --- Success, starting main processing loop!")
    dataLoop = juliaDataLoop(sysConfig, rover)

    wait(dataLoop)
    wait(sendLoop)
end

main()
