
export
    VariableRequest,
    FactorRequest,
    lsRequest,
    SetReadyRequest

struct VariableRequest
    label::String
    variableType::String
    N::Nullable{Int64}
    labels::Vector{String}
end

mutable struct FactorRequest
    factorType::String
    variables::Vector{String}
    measurement::Vector{Dict{String, Any}}
end

struct lsRequest
    variables::String
    factors::String
end

struct SetReadyRequest
    variables::Nullable{Vector{String}}
    isReady::Int
end
