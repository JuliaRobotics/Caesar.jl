
export
    VariableRequest,
    lsRequest

struct VariableRequest
    label::String
    variableType::String
    N::Nullable{Int64}
    labels::Vector{String}
end

struct lsRequest
    variables::String
    factors::String
    landmarks::String
end
