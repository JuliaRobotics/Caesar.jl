# multisession support for Caesar.jl



function multisessionquery(conn, session::T, multisessions::Vector{T}) where {T <: AbstractString}
  len = length(multisessions)
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
function parsemultisessionqueryresult!(lm2others::Dict{Symbol, Dict{Symbol, Int}}, cph)
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
      if !haskey(lm2others, sess)  lm2others[sess] = Dict{Symbol, Int}() end
      lm2others[sess][Symbol(res[1])] = res[2][j]
    end
  end
  nothing
end

"""
    getLandmOtherSessNeoIDs{T <: AbstractString}(::CloudGraph, session::T="", multisessions=Vector{T}())

Return dict of dict of Neo4j vertex IDs by session and landmark symbols.
"""
function getLandmOtherSessNeoIDs(cg::CloudGraph;
      session::T="",
      multisessions::Vector{T}=String[]  ) where {T <: AbstractString}
  #
  lm2others = Dict{Symbol, Dict{Symbol, Int}}()

  len = length(multisessions)
  len > 0 ? nothing : (return lm2others)

  if length(multisessions)==0
    cph = multisessionquery(cg.neo4j.connection, session, multisessions)
    parsemultisessionqueryresult!(lm2others, cph)
  else
    info("Ignoring multisession")
  end

  return lm2others
end


"""
    getprp2kde(::CloudGraph, neoids::Vector{Int}; N::Int=100)

Return PriorPoint2DensityNH with N points based on beliefs of neoids, and equal
share null hypothesis between length(neoids)+1 beliefs.
"""
function getprpt2kde(cloudGraph::CloudGraph, neoids::Vector{Int}; N::Int=100 )
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
  @show size(ptsN)

  nh = 1.0/(length(neoids)+1.0)
  ph = Float64[nh;1.0-nh]
  return PriorPoint2DensityNH(kde!(ptsN), ph)
end


"""
    getAllLandmarkNeoIDs(::Dict{Symbol, Dict{Symbol, Int}}, ::Symbol)

Return Vector{Int} of Neo4j vertex IDs relating to symbol, as listed in lm2others.
"""
function getAllLandmarkNeoIDs(lm2others::Dict{Symbol, Dict{Symbol,Int}}, slm::Symbol)
  mslmneoids = Int[]
  for (sess, vals) in lm2others
    if haskey(vals, slm)
      push!(mslmneoids, vals[slm])
    end
  end
  return mslmneoids
end


"""
    getLocalSubGraphMultisession{T <: AbstractString}(cg::CloudGraph, lm2others; session::T="", numneighbors::Int=0)

Return subgraph copy of type FactorGraph contaning values from session in lm2others, and Vector{Symbol} of primary
key symbols used for graph exstraction.
"""
function getLocalSubGraphMultisession(cg::CloudGraph,
            lm2others;
            session::T="",
            numneighbors::Int=0  ) where {T <: AbstractString}
  #
  res = Dict{Symbol, Int}()
  sfg = Caesar.initfg(sessionname=session, cloudgraph=cg)
  if length(lm2others) > 0
    for (sess,ms) in lm2others
      for (sym, neoid) in ms
        res[sym] = 0
      end
    end
    getVertNeoIDs!(cg, res, session=session)
    fullcurrneolist = collect(values(res))
    fetchsubgraph!(sfg, fullcurrneolist, numneighbors=numneighbors) # can set numneighbors=0
  end
  return sfg, collect(keys(res))
end


"""
    findExistingMSConstraints(fgl::FactorGraph)

Return Dict{Symbol, Int} of vertex symbol to Neo4j node ID of MULTISESSION constraints in this `fgl.sessionname`.
"""
function findExistingMSConstraints(fgl::FactorGraph)
  loadtx = transaction(fgl.cg.neo4j.connection)
  query =  "match (n:$(fgl.sessionname):MULTISESSION)
            return id(n), n.exVertexId, n.label"
  cph = loadtx(query, submit=true)

  # parse the response into a dictionary
  existingms = Dict{Symbol, Int}() # symlbl => neoid
  for res in cph.results[1]["data"]
    neoid, exvid, lbl = res["row"][1], res["row"][2], res["row"][3]
    existingms[Symbol(lbl)] = neoid
  end

  return existingms
end


"""
    removeReinsertMultisessionPrior!{T <: FunctorSingleton}(fgl::FactorGraph, exims::Dict{Symbol, Int}, prp2::T, sym::Symbol, uid::Int)

Return new multisession factor, symbol sym, inserted using prp2::PriorPoint2DensityNH using user ExVertex id (uid),
after checking existing multisession dict (exims::(exvsym=>neoid)) if any nodes should be removed before inserting new ones
on that graph node. Assuming just one multisession prior per node.
"""
function removeReinsertMultisessionPrior!{T <: FunctorSingleton}(fgl::FactorGraph,
        exims::Dict{Symbol, Int},
        prp2::T,
        sym::Symbol,
        uid::Int  )
  #
  # remove previous Multi session constraint on this sym vertex
  if haskey(exims, sym)
    info("Removing previous multisession neoid=$(exims[sym]) on sym=$(sym).")
    removeNeo4jID(fgl.cg, neoid=exims[sym])
    delete!(exims, sym)
  end
  # add new multisession constraint to the graph
  addFactor!(fgl, [sym], prp2, labels=["MULTISESSION"], uid=uid)
end


"""
    rmInstMultisessionPriors!(::CloudGraph; session<:AbstractString=, multisessions::Vector{<:AbstractString}= )
"""
function rmInstMultisessionPriors!(cloudGraph::CloudGraph;
      session::T="NA",
      multisessions::Vector{T}=String[]  ) where {T <: AbstractString}
  #
  session!="NA" ? nothing : error("Please specify a valid session, currently = $(session)")

  multisessionsl = Vector{String}(setdiff(multisessions, [session]))
  length(multisessionsl) > 0 ? nothing : (return nothing)
  # get the landmarks of interest from neighboring sessions
  lm2others = getLandmOtherSessNeoIDs(cloudGraph,
                  session=session,multisessions=multisessionsl)
  #

  # grab local subgraph using NeoIDs
  sfg, lms = getLocalSubGraphMultisession(cloudGraph, lm2others,
                  session=session, numneighbors=1)
  #

  # get dict(sym => neoid) of exiting multisession constraints which may be removed during this update
  exims = findExistingMSConstraints(sfg)
  # get highest factor exvid
  mfn4jid = getmaxfactorid(cloudGraph.neo4j.connection, session)

  println("Multisession constraints in $(session), from $(multisessionsl), on $(lms)")
  for sym in lms
    neoids = getAllLandmarkNeoIDs(lm2others, sym)
    prp2 = getprpt2kde(cloudGraph, neoids, N=100 )
    mfn4jid += 1
    factorms = removeReinsertMultisessionPrior!(sfg, exims, prp2, sym, mfn4jid)
  end
  nothing
end


function removeMultisessions!(cloudGraph::CloudGraph; session::AbstractString="NA")
  session!="NA" ? nothing : error("Please specify a valid session, currently = $(session)")

  loadtx = transaction(cloudGraph.neo4j.connection)
  query =  "match (n:$(session):MULTISESSION)
            detach delete n
            return count(n)"
  cph = loadtx(query, submit=true)
  commit(loadtx)
  return cph
end




#
