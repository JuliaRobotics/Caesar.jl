# multisession support for Caesar.jl



function multisessionquery{T <: AbstractString}(conn, session::T, multisessions::Vector{T})
  len = length(multisessions)
  len > 0 ? nothing : (return lm2others)

  loadtx = transaction(conn)
  # construct the query
  query = "match (m:$(session):LANDMARK) "*
          "with m.label as mlb "*
          "order by mlb asc "*
          "match (n:LANDMARK) "*
          "where n.label = mlb "*
          "and (";
  for i in 1:len
    ots = multisessions[i]
    query = query*"n:$(ots) "
    query = i < len ? query*"or " : query
  end
  query = query*") "*
                "with distinct n.label as lbl, collect(id(n)) as others, collect(labels(n)) as nlb "*
                "return lbl, others, nlb";

  loadtx(query, submit=true)
end

# multisession must have session label starting with SESS* in each query response line
function parsemultisessionqueryresult!(lm2others::Dict{Symbol, Vector{Tuple{Symbol, Int}}}, cph)
  i = 0
  # @show cph.results[1]["data"]
  for i in 1:length(cph.results[1]["data"])
    res = cph.results[1]["data"][i]["row"]
    for j in 1:length(res[2])
      sess = ""
      for lb in res[3][j]
        if lb[1:4] == "SESS"
          sess = Symbol(lb)
        end
      end
      if !haskey(lm2others, sess)  lm2others[sess] = Vector{Tuple{Symbol, Int}}() end
      push!(lm2others[sess], (Symbol(res[1]), res[2][j]) )
    end
  end
  nothing
end

function getLandmOtherSessNeoIDs{T <: AbstractString}(cg::CloudGraph;
      session::T="",
      multisessions::Vector{T}=String[]  )
  #
  lm2others = Dict{Symbol, Vector{Tuple{Symbol, Int}}}()

  cph = multisessionquery(cg.neo4j.connection, session, multisessions)
  parsemultisessionqueryresult!(lm2others, cph)

  return lm2others
end



function getprp2kde(cloudGraph::CloudGraph, neoids::Vector{Int}; N::Int=100 )
  #
  # P = BallTreeDensity[]
  PTS = Array{Float64,2}()
  for fielem in neoids # lm2others[lm]
    cv = CloudGraphs.get_vertex(cloudGraph, fielem)
    vert = cloudVertex2ExVertex(cv)
    pts = getVal(vert)
    PTS = size(PTS,2) == 0 ? pts' : [PTS; pts']
    # push!(P, kde!(pts))
  end
  PTS = PTS';

  l = collect(1:size(PTS,2))
  shuffle!(l)

  # TODO -- not up sampling if N is bigger than size(PTS)
  ptsN = PTS[:,l[1:N]];

  nh = 1.0/(length(neoids)+1.0)
  ph = Float64[nh;1.0-nh]
  return PriorPoint2DensityNH(kde!(ptsN), ph)
end
