module Multisession

using DocStringExtensions
using Combinatorics
using Caesar
using Neo4j, CloudGraphs

export
  getSessionsForEnvironment,
  getEnvironmentPrimeLandmarkNodes,
  getSessionLandmarks,
  getFederatedGraphElements,
  buildPrimeLandmarksAndFactors,
  getMultiSessionFg,
  updateLandmarkProducts

"""
    $(SIGNATURES)

Return all binary pair combinations of sessions and return a Vector::Tuple{String, String}.

Example
-------
```julia
str = ["a";"b";"c";"d"]
pstr = stringPairs(str)
@assert length(pstr) == 6
```
"""
function stringPairs(strs::Vector{<:AbstractString} )::Vector{Tuple{String, String}}
  ret = Tuple{String,String}[]
  for co in combinations( strs )
    if length(co) == 2
      push!(ret, (co[1], co[2]))
    end
  end
  return ret
end

"""
  $(SIGNATURES)

Gets a vector of session IDs that are related to an environment.
"""
function getSessionsForEnvironment(
    connection::Neo4j.Connection,
    environment::String)::Vector{String}
  query = "match (env:ENVIRONMENT:$environment)-[SESSION]->(s:SESSION) return s.id"
  cph, = Caesar.executeQuery(connection, query)
  return map(s -> s["row"][1], cph.results[1]["data"])
end

"""
  $(SIGNATURES)

Get a dictionary (label => Neo4j node) of all prime nodes for a given set of sessions and an environment.
"""
function getEnvironmentPrimeLandmarkNodes(cloudGraph::CloudGraph, sessions::Vector{String}, environment::String)::Dict{String, Neo4j.Node}
  # TODO - refactor prime labels as a special snowflake - m or something like `:m14` is better -- still need to test and upgrade IIF for this
  query = "match (node:MULTISESSION:LANDMARK) where ("*join("(node:".*sessions.*")", " or ")*") and node.environment = \"$environment\" return id(node)"
  cph, = Caesar.executeQuery(cloudGraph.neo4j.connection, query)
  existPrimeNodes = map(node -> getnode(cloudGraph.neo4j.graph, node["row"][1]), cph.results[1]["data"])
  labels = map(n -> getnodeproperty(n, "label"), existPrimeNodes)
  # Dictionary of all existing prime nodes...
  existPrimeNodes = Dict(labels .=> existPrimeNodes)
  return existPrimeNodes
end

"""
  $(SIGNATURES)

Gets all landmark IDs for a given session, excluding multisession.
"""
# TODO: This should be a generic function...
function getSessionLandmarks(
    conn::Neo4j.Connection,
    sessionId::String,
    robotId::String,
    userId::String)::Vector{String}
  query = "match (landmark:$sessionId:$robotId:$userId:LANDMARK) where NOT landmark:MULTISESSION return landmark.label"
  cph, = Caesar.executeQuery(conn, query)
  return map(s -> s["row"][1], cph.results[1]["data"])
end

"""
  $(SIGNATURES)

Very specific method to check if a prime factor exists between a session landmark and a prime node.
Prime node is identified by NeoNode ID.
"""
function _doesPrimeFactorExistByForSessionLandmark(
    conn::Neo4j.Connection,
    sessionId::String,
    robotId::String,
    userId::String,
    primeLandmarkNeoNodeId::Int)::Bool
  query = "match (node:LANDMARK:$sessionId:$robotId:$userId)-[:DEPENDENCE]-(f:FACTOR)-[:DEPENDENCE]-(m:MULTISESSION) where id(m)=$(primeLandmarkNeoNodeId) return id(node)"
  cph, = Caesar.executeQuery(conn, query)
  return length(cph.results[1]["data"]) > 0 # If it exists return true.
end

"""
  $(SIGNATURES)

Returns the tuples of the prime landmark -> prime factor -> session landmark for all existing prime factors.
Tuple elements: mid, mlabel, fid, flabel, lid, llabel
May contain duplicates elements (e.g. two prime factors for one prime landmark).
"""
function getFederatedGraphElements(
    connection::Neo4j.Connection,
    sessions::Vector{String},
    environment::String)::Vector{Tuple}
  query = "match (m:LANDMARK:MULTISESSION)--(f:FACTOR:MULTISESSION)--(l:LANDMARK) where "*(prod("m:".*sessions.*" or ")[1:(end-4)])*" and m.environment=\"$environment\" return id(m), m.label, id(f), f.label, id(l), l.label"
  cph, = Caesar.executeQuery(connection, query)

  elements = Vector{Tuple}()
  for data in cph.results[1]["data"]
    #Tuple elements: mid, mlabel, fid, flabel, lid, llabel
    push!(elements, (data["row"][1], Symbol(data["row"][2]), data["row"][3], Symbol(data["row"][4]), data["row"][5], Symbol(data["row"][6])))
  end
  return elements
end

"""
  $(SIGNATURES)

McFlurry step.
Builds the prime landmarks and prime factors for a given environment and sessions (if they don't exist).
Returns all prime landmarks.
"""
function buildPrimeLandmarksAndFactors(
    cloudGraph::CloudGraph,
    sessionLandmarks::Dict{String, Vector{String}},
    robotId::String,
    userId::String,
    environment::String)::Dict{String, Neo4j.Node}
  @debug "  - Checking and updating each combination of sessions for same landmarks..."
  sessions = collect(keys(sessionLandmarks))
  sessionPairs = stringPairs(sessions)
  primeLandmarkNodes = getEnvironmentPrimeLandmarkNodes(cloudGraph, sessions, environment)
  for (s1,s2) in sessionPairs
    @info "    - Creating landmark links between '$s1' and '$s2'"
    # TODO: Find a better way to for landmark equivalence check
    # TODO: Check that the landmarks are the same type
    commonLandmarks = intersect(sessionLandmarks[s1], sessionLandmarks[s2])
    @info "    - Common landmark set: $commonLandmarks"

    for commonLandmark in commonLandmarks
      @info "    - Checking common landmark $commonLandmark..."
      # Make the prime variable if it doesn't exist
      sym = Symbol("l"*string(parse(Int, commonLandmark[2:end])+100000))
      if !haskey(primeLandmarkNodes, String(sym))
        # TODO: be generic about which variables and factor types to use
        # Make a prime and add to db+list
        @info "    - Adding a prime landmark variable $sym..."
        fg = initfg(cloudgraph=cloudGraph, robotname=robotId, username=userId, sessionname=s1)

        # Check the softtypes to make sure they match.
        # TODO: FRACK GETTING AGAIN... YEESH
        land1 = Caesar.getCloudVert(cloudGraph, s1, robotId, userId, sym=Symbol(commonLandmark))
        land2 = Caesar.getCloudVert(cloudGraph, s2, robotId, userId, sym=Symbol(commonLandmark))
        if typeof(land1.packed.softtype) != typeof(land2.packed.softtype)
            @error "Softtypes don't match for $s1:$sym (got $(typeof(land1.packed.softtype))) and $s2:$sym (got $(typeof(land2.packed.softtype)))"
            continue
        end
        addVariable!(fg, sym, deepcopy(land1.packed.softtype), tags=[:MULTISESSION; :LANDMARK; Symbol(s1); Symbol(s2); Symbol(userId); Symbol(robotId)], uid=100000)
        # NOTE: Until DFG, all prime variables have exVertexId of 100000
        # Add the environment property
        setnodeproperty(getnode(cloudGraph.neo4j.graph, fg.cgIDs[100000]), "environment", environment)
        @debug "    - Added prime landmark!"
        # primeLandmarkNodes[string(sym)] = primeLandmarkNodes
        # Repull the existing prime nodes!
        primeLandmarkNodes = getEnvironmentPrimeLandmarkNodes(cloudGraph, sessions, environment)
      else
        @info "    - Prime landmark $sym already exists, making sure it has all the labels!"
        addnodelabels(primeLandmarkNodes[String(sym)], [s1, s2])
      end

      # check if factor already exists
      prime = primeLandmarkNodes[string(sym)]
      for s in [s1, s2]
        if !_doesPrimeFactorExistByForSessionLandmark(cloudGraph.neo4j.connection, s, robotId, userId, prime.id)
          @info "    - Adding a factor between session landmark '$commonLandmark' and prime landmark '$sym' for session $s..."
          # add new factor that between session landmark and prime landmark
          fg = initfg(cloudgraph=cloudGraph, robotname=robotId, username=userId, sessionname=s)
          syms = [Symbol(commonLandmark);sym]
          @debug "    - Copying local symbols $syms..."
          Caesar.subLocalGraphCopy!(fg, syms, neighbors=0, reqbackendset=false, reqSolvable=false, includeMultisession=true)

          # Note: this shouldn't be necessary because the check is done in the landmark creation, but doing just in case here too.
          @show sessionLandType = getData(getVert(fg, syms[1], api=localapi)).softtype
          @show primeLandType = getData(getVert(fg, syms[2], api=localapi)).softtype
          if typeof(sessionLandType) != typeof(primeLandType)
              @error "Softtypes don't match for $syms (got $(typeof(sessionLandType))) and (got $(typeof(primeLandType)))"
              continue
          end
          newFactorType = string(typeof(sessionLandType).name)*string(typeof(primeLandType).name)

          @debug "    - Trying to create a factor linking $syms of type $newFactorType ..."
          # How do i create a Point2Point2 or Pose2Pose2 generically.
          newPrimeFactor = Point2Point2(MvNormal(zeros(2), 1e-4*Matrix(LinearAlgebra.I, 2,2) ))
          @warn "Currently would like a factor of type $newFactorType, but hard-coded to create Point2Point2's. Please fix in FederatedSolving.jl!"
          addFactor!(fg, syms, newPrimeFactor, uid=100001, tags=[:FACTOR; :MULTISESSION; Symbol(userId); Symbol(robotId); Symbol(s)], autoinit=false)
          # NOTE: Until DFG, all prime variables have exVertexId of 100000
          # Add the environment property
          setnodeproperty(getnode(cloudGraph.neo4j.graph, fg.cgIDs[100001]), "environment", environment)
        else
          @debug "    - Factor link already exists, continuing..."
        end
      end
    end
  end
  return primeLandmarkNodes
end

"""
  $(SIGNATURES)

Create a Caesar FactorGraph that contains all landmarks and factors in a multisession
federated solve.
"""
function getMultiSessionFg(
    cloudGraph::CloudGraph,
    sessions::Vector{String},
    environment::String)
# #
  fg = initfg(cloudgraph=cloudGraph)

  # 1. Get all federated elements - Vector of tuples (mid,  mlabel, fid, flabel, lid, llabel)
  federatedGraphElements = getFederatedGraphElements(cloudGraph.neo4j.connection, sessions, environment)

  localIndex = 1
  for (mid, mlabel, fid, flabel, lid, llabel) in federatedGraphElements

    # make the local index unique
    origlabel = llabel
    llabel = Symbol(string(llabel, "_", localIndex))
    # If prime landmark not in FG, add it
    if !(mid in values(fg.IDs))
      mvert = CloudGraphs.get_vertex(fg.cg, mid, false)
      addVariable!(fg, mlabel, mvert.packed.softtype, api=IIF.localapi, uid=mid)
      getVert(fg, mlabel, api=localapi).attributes["origlabel"] = mlabel
      # Manually add the cloud node IDs
      push!(fg.cgIDs, mvert.neo4jNodeId => mvert.neo4jNodeId)
    end
    # If session landmark not in FG, add it
    if !(lid in values(fg.IDs))
      lvert = CloudGraphs.get_vertex(fg.cg, lid, false)
      addVariable!(fg, llabel, lvert.packed.softtype, api=IIF.localapi, uid=lid)
      getVert(fg, llabel, api=localapi).attributes["origlabel"] = origlabel
      Caesar.setValKDE!(fg, llabel, kde!(lvert.packed.val), api=IIF.localapi)
      # Manually add the cloud node IDs
      push!(fg.cgIDs, lvert.neo4jNodeId => lvert.neo4jNodeId)
      # Keep local label index unique
      localIndex += 1
    end
    # If prime factor (l1 prime -- l1 session) not in FG, add it
    if !(fid in values(fg.fIDs))
      vm = fg.g.vertices[mid]
      vl = fg.g.vertices[lid]
      fvert = CloudGraphs.get_vertex(fg.cg, fid, false)
      addFactor!(fg, [llabel; mlabel], fvert.packed.fnc.usrfnc!, api=IIF.localapi, uid=fid, autoinit=false)
    end
  end
  return fg
end

"""
$(SIGNATURES)

Updates the specified list of landmarks using local products.
Returns a symbol dictionary of the provided landmarks and a tuple of (new KDE, KDE propsals),
which is useful for plotting with plotKDE.
"""
function updateLandmarkProducts(
    fg::G,
    landmarkSymbols::Vector{Symbol},
    shouldFreeze::Bool)::Dict{Symbol, Tuple{BallTreeDensity, Vector{BallTreeDensity}}} where G <: AbstractDFG
  retDict = Dict{Symbol, Tuple{BallTreeDensity, Vector{BallTreeDensity}}}()
  for landmarkSymbol in landmarkSymbols
    @debug "Calculating local product for $landmarkSymbol..."

    # Update calculate the local product and persist it
    newkde, kdeproposals = IIF.localProduct(fg, landmarkSymbol, api=IIF.localapi)

    # SetValKDE does not do what we need here - do by hand...
    # Get local, update cloud.
    vert = getVert(fg, landmarkSymbol, api=IIF.localapi)
    data = getData(vert)
    data.val = getPoints(newkde)
    if shouldFreeze
      @debug "Freezing $landmarkSymbol..."
      # Freeze the landmark (TODO: May be overcooked, possibly refactore)
      data.ismargin = true
    end
    vert.attributes["label"] = vert.attributes["origlabel"]
    setData!(vert, data)

    @debug "Updating $landmarkSymbol on server..."
    # Fix the label before writing back
    dlapi.updatevertex!(fg, vert)
    #setValKde is not right call :/
    # setValKDE!(fg, landmarkSymbol, newkde, api=IIF.dlapi)

    push!(retDict, landmarkSymbol=>(newkde, kdeproposals))
  end
  return retDict
end

return nothing

end
