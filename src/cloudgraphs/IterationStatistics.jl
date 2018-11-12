using Dates

export IterationStatistics

"""
A simple structure containing performance stats from the iteration.
"""
mutable struct IterationStatistics
    startTimestamp::DateTime
    endTimestamp::DateTime
    numNodes::Int
    result::String
    additionalInfo::Dict{String, String}
    IterationStatistics(startTimestamp, endTimestamp, numNodes, result, additionalInfo::Dict{String, String} = Dict{String, String}()) = new(startTimestamp, endTimestamp, numNodes, result, additionalInfo)
    IterationStatistics() = new(Dates.now(), Dates.now(), 0, "N/A")
end
