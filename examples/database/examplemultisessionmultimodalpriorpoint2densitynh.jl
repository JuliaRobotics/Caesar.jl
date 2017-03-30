# example of creating multi-session, multi-modal PriorPoint2DensityNH

using Caesar
using Neo4j
using CloudGraphs
using RoME
using KernelDensityEstimate

function getLandmOtherSessNeoIDs{T <: AbstractString}(cg::CloudGraph;
      session::T="",
      multisessions::Vector{T}=String[]  )
  #
  lm2others = Dict{Symbol, Vector{Int}}()
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
                "with distinct n.label as lbl, collect(id(n)) as others "*
                "return lbl, others ";

  cph = loadtx(query, submit=true)
  for res in cph.results[1]["data"]
    lm2others[Symbol(res["row"][1])] = res["row"][2]
  end
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



N = 100

# standard setup, auth from file
include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
addrdict["num particles"] = "$(N)"
addrdict["session"] = "SESSTURT21"
cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)

# convenience variables
conn = cloudGraph.neo4j.connection
session = addrdict["session"]


multisessions = ["SESSTURT38";"SESSTURT45"]


# get the landmarks of interest from neighboring sessions
lm2others = getLandmOtherSessNeoIDs(cloudGraph, session=session,
                      multisessions=multisessions)


#


lm = :l200583

prp2d = getprp2kde(cloudGraph, lm2others[lm], N=N )
# prp2d.nullhypothesis

# addFactor!(fgl, [lm], prp2d, ready=ready)


plotKDE(P)
plotKDE([kde!(ptsN);P],c=["red";"green";"blue"],levels=3)



#
