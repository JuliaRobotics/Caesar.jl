
import Base: convert

export
    VariableRequest,
    FactorRequest,
    lsRequest,
    setSolvableRequest

struct VariableRequest
    label::String
    variableType::String
    N::Union{Int64, Nothing}
    labels::Vector{String}
end

mutable struct FactorRequest
    # factorType::String # This is now in the factor itself.
    variables::Vector{String}
    factorType::String
    factor::Dict{String, Any}
end

struct lsRequest
    variables::String
    factors::String
end

struct setSolvableRequest
    variables::Union{Vector{String}, Nothing}
    isSolvable::Int
end
