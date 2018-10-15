
export
    VariableRequest,
    FactorRequest,
    lsRequest

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
