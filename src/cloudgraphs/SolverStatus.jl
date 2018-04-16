using Base.Dates

export SolverStatus

"""
A simple structure for monitoring a running solver.
"""
mutable struct SolverStatus
    id::String
    host::String
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
    SolverStatus() = new(string(Base.Random.uuid4()), "", false, "", "", "", 0, "", string(unix2datetime(time())), "", "", "", 0)
    SolverStatus(id::String, host::String, isAttached::Bool, userId::String, robotId::String, sessionId::String, iteration::Int64, currentStep::String, startedUpTimestamp::String, attachedSessionTimestamp::String, detachedSessionTimestamp::String, lastUpdatedTimestamp::String, lastIterationDurationSeconds::Float64) = new(id, host, isAttached, userId, robotId, sessionId, iteration, currentStep, startedUpTimestamp, attachedSessionTimestamp, detachedSessionTimestamp, lastUpdatedTimestamp, lastIterationDurationSeconds)
end
