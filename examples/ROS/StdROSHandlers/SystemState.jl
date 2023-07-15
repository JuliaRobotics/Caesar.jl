# using DocStringExtensions

"""
    ::SystemState

Quick placeholder for the system state - we're going to use timestamps to align all the data.
"""
Base.@kwdef mutable struct SystemState
    curtimestamp::Float64 = -1000
    cur_variable::Union{Nothing, String} = nothing
    prv_variable::String = ""
    var_index::Int = 0
    lidar_scan_index::Int = 0
    """ add variables and factors to cloud servers """
    pushNva::Bool = true
    """ push data blobs to cloud servers """
    pushBlobs::Bool = false
    """ stripe radar points by taking every n-th element (TODO remove legacy, use stripe pose keyframe option)"""
    stripeClouds::Int = 1
    """ stripe scans by taking every n-th element (TODO remove here as future via options) """
    stripeKeyframe::Int = 1
    """ general containers specific to user requirements, workspace for customer specific needs """
    workspace::Dict{String,Any} = Dict{String,Any}()
end