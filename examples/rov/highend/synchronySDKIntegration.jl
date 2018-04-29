# SynchronySDK integration file

using SynchronySDK
using DocStringExtensions # temporary while $(SIGNATURES) is in use in this file

const Syncr = SynchronySDK

"""
    SyncrSLAM

An object definition containing the require variables to leverage the server side SLAM solution per user, robot, and session.
"""
mutable struct SyncrSLAM
  robotId::AbstractString
  sessionId::AbstractString
  syncrconf::Union{Void, SynchronySDK.SynchronyConfig}
  robot
  session

  # Constructor
  SyncrSLAM(;
    userId::AbstractString="",
    robotId::AbstractString="",
    sessionId::AbstractString="",
    syncrconf::Union{Void, SynchronySDK.SynchronyConfig}=nothing,
    robot=nothing,
    session=nothing
   ) = new(
        userId,
        robotId,
        sessionId,
        syncrconf,
        robot,
        session
      )
end


"""
$(SIGNATURES)

Get  Synchrony configuration, default filepath location assumed as `~/Documents/synchronyConfig.json`.
"""
function loadSyncrConfig(;
            filepath::AS=joinpath(ENV["HOME"],"Documents","synchronyConfig.json")
         ) where {AS <: AbstractString}
  #
  println(" - Retrieving Synchrony Configuration...")
  configFile = open(filepath)
  configData = JSON.parse(readstring(configFile))
  close(configFile)
  Unmarshal.unmarshal(SynchronyConfig, configData) # synchronyConfig =
end

"""
$(SIGNATURES)

Confirm that the robot already exists, create if it doesn't.
"""
function syncrRobot(
           synchronyConfig::SynchronyConfig,
           robotId::AS
         ) where {AS <: AbstractString}
  #
  println(" - Creating or retrieving robot '$robotId'...")
  robot = nothing
  if(SynchronySDK.isRobotExisting(synchronyConfig, robotId))
      println(" -- Robot '$robotId' already exists, retrieving it...")
      robot = getRobot(synchronyConfig, robotId)
  else
      # Create a new one
      println(" -- Robot '$robotId' doesn't exist, creating it...")
      newRobot = RobotRequest(robotId, "My New Bot", "Description of my neat robot", "Active")
      robot = createRobot(synchronyConfig, newRobot)
  end
  println(robot)
  robot
end

"""
$(SIGNATURES)

Get sessions, if it already exists, add to it.
"""
function syncrSession(
            synchronyConfig::SynchronyConfig,
            robotId::AS,
            sessionId::AS
         ) where {AS<: AbstractString}
  #
  println(" - Creating or retrieving data session '$sessionId' for robot...")
  session = nothing
  if(SynchronySDK.isSessionExisting(synchronyConfig, robotId, sessionId))
      println(" -- Session '$sessionId' already exists for robot '$robotId', retrieving it...")
      session = getSession(synchronyConfig, robotId, sessionId)
  else
      # Create a new one
      println(" -- Session '$sessionId' doesn't exist for robot '$robotId', creating it...")
      newSessionRequest = SessionDetailsRequest(sessionId, "A test dataset demonstrating data ingestion for a   wheeled vehicle driving in a hexagon.")
      session = createSession(synchronyConfig, robotId, newSessionRequest)
  end
  println(session)
  session
end

"""
$(SIGNATURES)

Intialize the `sslaml` object using configuration file defined in `syncrconfpath`.
"""
function initialize!(sslaml::SyncrSLAM;
            syncrconfpath::AS=joinpath(ENV["HOME"],"Documents","synchronyConfig.json")
         ) where {AS <: AbstractString}
  # 1. Get a Synchrony configuration
  sslaml.syncrconf = loadSyncrConfig(filepath=syncrconfpath)

  # 2. Confirm that the robot already exists, create if it doesn't.
  sslaml.robot = syncrRobot(sslaml.syncrconf, sslaml.robotId)

  # 3. Create or retrieve the session.
  sslaml.session = syncrSession(sslaml.syncrconf, sslaml.robotId, sslaml.sessionId)

  nothing
end
