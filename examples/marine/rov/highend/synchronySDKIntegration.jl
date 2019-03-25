# GraffSDK integration file

using GraffSDK
using DocStringExtensions # temporary while $(TYPEDSIGNATURES) is in use in this file

const Graff = GraffSDK

"""
    GraffSLAM

An object definition containing the require variables to leverage the server side SLAM solution per user, robot, and session.
"""
mutable struct GraffSLAM
  robotId::AbstractString
  sessionId::AbstractString
  graffconf::Union{Nothing, GraffSDK.GraffConfig}
  robot
  session

  # Constructor
  GraffSLAM(
    robotId::AbstractString,
    sessionId::AbstractString,
    graffconf::Union{Nothing, GraffSDK.GraffConfig};
    robot=nothing,
    session=nothing
   ) = new(
        robotId,
        sessionId,
        graffconf,
        robot,
        session
      )
end


"""
$(TYPEDSIGNATURES)

Get  Graff configuration, default filepath location assumed as `~/Documents/graffConfig.json`.
"""
function loadGraffConfig(;
            console::Vector{Symbol}=[],
            filepath::AS=joinpath(ENV["HOME"],"Documents","graffConfig.json")
         ) where {AS <: AbstractString}
  #
  println(" - Retrieving Graff Configuration...")
  configFile = open(filepath)
  configData = JSON.parse(readstring(configFile))
  close(configFile)
  cfg = Unmarshal.unmarshal(GraffConfig, configData) # graffConfig =

  :userId in console ? (println("userId: "); cfg.userId = readline(stdin);) : nothing
  :robotId in console ? (println("robotId: "); cfg.robotId = readline(stdin);) : nothing
  :sessionId in console ? (println("sessionId: "); cfg.sessionId = readline(stdin);) : nothing

  return cfg
end

"""
$(TYPEDSIGNATURES)

Confirm that the robot already exists, create if it doesn't.
"""
function graffRobot(
           graffConfig::GraffConfig,
           robotId::AS
         ) where {AS <: AbstractString}
  #
  println(" - Creating or retrieving robot '$robotId'...")
  robot = nothing
  if(GraffSDK.isRobotExisting(graffConfig, robotId))
      println(" -- Robot '$robotId' already exists, retrieving it...")
      robot = getRobot(graffConfig, robotId)
  else
      # Create a new one
      println(" -- Robot '$robotId' doesn't exist, creating it...")
      newRobot = RobotRequest(robotId, "My New Bot", "Description of my neat robot", "Active")
      robot = addRobot(graffConfig, newRobot)
  end
  println(robot)
  robot
end

"""
$(TYPEDSIGNATURES)

Get sessions, if it already exists, add to it.
"""
function graffSession(
            graffConfig::GraffConfig,
            robotId::AS,
            sessionId::AS
         ) where {AS<: AbstractString}
  #
  println(" - Creating or retrieving data session '$sessionId' for robot...")
  session = nothing
  if(GraffSDK.isSessionExisting(graffConfig, robotId, sessionId))
      println(" -- Session '$sessionId' already exists for robot '$robotId', retrieving it...")
      session = getSession(graffConfig, robotId, sessionId)
  else
      # Create a new one
      println(" -- Session '$sessionId' doesn't exist for robot '$robotId', creating it...")
      newSessionRequest = SessionDetailsRequest(sessionId, "Submersible vehicle.", "Pose3", false)
      session = addSession(graffConfig, robotId, newSessionRequest)
  end
  println(session)
  session
end

"""
$(TYPEDSIGNATURES)

Intialize the `sslaml` object using configuration file defined in `graffconfpath`.
"""
function initialize!(sslaml::GraffSLAM;
            graffconfpath::AS=joinpath(ENV["HOME"],"Documents","graffConfig.json")
         ) where {AS <: AbstractString}
  # 1. Get a Graff configuration
  sslaml.graffconf = loadGraffConfig(filepath=graffconfpath)

  # 2. Confirm that the robot already exists, create if it doesn't.
  sslaml.robot = graffRobot(sslaml.graffconf, sslaml.robotId)

  # Make sure that this session is not already populated
  if isSessionExisting(slam_client.graffconf, sslaml.robotId, sslaml.sessionId)
      @warn "There is already a session named '$sessionId' for robot '$robotId'. This example will fail if it tries to add duplicate nodes. We strongly recommend providing a new session name."
      print(" Should we delete it? [Y/N] - ")
      if lowercase(strip(readline(stdin))) == "y"
          deleteSession(sslaml.graffconf, sslaml.robotId, sslaml.sessionId)
          println(" -- Session '$sessionId' deleted!")
      else
          @warn "Okay, but this may break the example! Caveat Userus!"
      end
  end

  # 3. Create or retrieve the session.
  sslaml.session = graffSession(sslaml.graffconf, sslaml.robotId, sslaml.sessionId)

  nothing
end
