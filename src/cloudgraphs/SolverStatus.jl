using Base.Dates

export SolverStatus

"""
A simple structure for monitoring a running solver.
"""
mutable struct SolverStatus
    isAttached::Bool
    userId::String
    robotId::String
    sessionId::String
    iteration::Int64
    currentStep::String
    startedUpTimestamp::String
    attachedSessionTimestamp::String
    detachedSessionTimestamp::String
    lastUpdatedTimestamp::String
    lastIterationDurationSeconds::Float64
    SolverStatus() = new(false, "", "", "", 0, "", string(unix2datetime(time())), "", "", "", 0)
end
