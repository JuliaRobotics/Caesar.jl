# example of creating multi-session, multi-modal PriorPoint2DensityNH

using Caesar
using Neo4j
using CloudGraphs
using RoME
using IncrementalInference
using KernelDensityEstimate


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

function fetchsubgraph!(fgl::FactorGraph,
          cvs::Vector{CloudGraphs.CloudVertex};
          numneighbors::Int=0 )
          # overwrite::Bool=false  )
  # recursion termination condition
  numneighbors >= 0 ? nothing : (return nothing)

  for cv in cvs
    # test if these are already in fgl
    if !hasval(fgl.cgIDs, cv.neo4jNodeId) # may have been inserted as previous neighbor
      # add this vert to graph
      @show "inserting", cv.neo4jNodeId, cv.exVertexId
      insertnodefromcv!(fgl, cv)

      # recursive call on neighbors here
      neicvs = CloudGraphs.get_neighbors(fgl.cg, cv, needdata=true)
      fetchsubgraph!(fgl, neicvs; numneighbors=numneighbors-1 )
      # add edges associated with the neighbors

      if numneighbors-1 >= 0
        for cvn in neicvs
          @show "edge", cv.exVertexId, cvn.exVertexId
          checkandinsertedges!(fgl, cv.exVertexId, cvn, ready=1, backendset=1)
          # makeAddEdge!(fgl, fgl.g.vertices[cv.exVertexId], fgl.g.vertices[cvn.exVertexId], saveedgeID=false)
        end
      end
    end

  end
end

function fetchsubgraph!(fgl::FactorGraph,
          neoids::Vector{Int};
          numneighbors::Int=0 )
          # overwrite::Bool=false  )
  #
  for nid in neoids
    # test if these are already in fgl
    if !hasval(fgl.cgIDs, nid)
      cv = CloudGraphs.get_vertex(fgl.cg, nid, false)
      fetchsubgraph!(fgl, [cv], numneighbors=numneighbors )
    end
  end
  nothing
end


sess = :SESSTURT38
dsfg = Dict{Symbol, FactorGraph}()

fullneoidlist = Vector{Int}()
for (vsym,neoid) in lm2others[sess]
  fullneoidlist = union(fullneoidlist, neoid)
end

# grab local subgraph using NeoIDs
dsfg[sess] = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)

# fetchsubgraph!(fg, [199436], numneighbors=2)



fetchsubgraph!(dsfg[sess], fullneoidlist, numneighbors=3)


writeGraphPdf(dsfg[sess])

run(`evince fg.pdf`)






#
