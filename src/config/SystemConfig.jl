using FileIO
using CloudGraphs

import JSON, Unmarshal

export SystemConfig, CaesarConfig, CaesarWebServerConfig, VisualizationConfig
export decodeSysConfig, encode, readSystemConfigFile

# Validation regexs
_alphaRegex1 = r"^[a-zA-Z0-9]{1,64}$"
_alphaRegex0 = r"^[a-zA-Z0-9]{0,64}$"

# Probably going add more to this in the future so separating it out
struct CaesarWebServerConfig
    port::Int64
end

struct VisualizationConfig
    drawDepth::Bool
    cleanSlamInDb::Bool
    drawEdges::Bool
end

struct CaesarConfig
    numParticles::Int64
    multiSession::Vector{AbstractString}
    webserverConfig::CaesarWebServerConfig
    visualizationConfig::VisualizationConfig
end

mutable struct SystemConfig
    cloudGraphsConfig::CloudGraphConfiguration
    caesarConfig::CaesarConfig
end

"""
Returns the system config from the given JSON
"""
function decodeSysConfig(jsonData::String)::SystemConfig
    return Unmarshal.unmarshal(SystemConfig, JSON.parse(jsonData))
end

"""
Returns encoded string for SystemConfig
"""
function encode(systemConfig::SystemConfig)::String
    return JSON.json(systemConfig)
end

"""
Read a config file from filesystem
"""
function readSystemConfigFile(fileName::String)
    try
        # Read the configuration
        f = open(fileName, "r")
        sysConfig = decodeSysConfig(readstring(f))
        close(f)

        return sysConfig
    catch e
        error("Could not read system configuration file - $e")
    end
end
