# integration code for database usage via CloudGraphs.jl

import IncrementalInference: getVert, getfnctype, ls

export
  executeQuery,
  listAllVariables,
  getCloudVert,
  getfnctype,
  usecloudgraphsdatalayer!,
  standardcloudgraphsetup,
  consoleaskuserfordb,
  registerGeneralVariableTypes!,
  fullLocalGraphCopy!,
  subLocalGraphCopy!,
  removeGenericMarginals!,
  setBackendWorkingSet!,
  setDBAllReady!,
  getExVertFromCloud,
  getAllExVertexNeoIDs,
  getPoseExVertexNeoIDs,
  getFirstPose,
  getfirstpose,
  getLastPose,
  copyAllNodes!,
  copyAllEdges!,
  registerCallback!,
  updateFullCloudVertData!,
  #loading frtend generated fg
  getnewvertdict,
  mergeValuesIntoCloudVert!,
  recoverConstraintType,
  populatenewvariablenodes!,
  populatenewfactornodes!,
  updatenewverts!,
  resetentireremotesession,
  appendvertbigdata!,
  # visualization exports
  getPointCloudFromKinect,
  getPointCloudFromBSON,
  deleteServerSession!


"""
    $(SIGNATURES)

Run Neo4j Cypher queries on the cloudGraph database, andreturn Tuple with the
unparsed (results, loadresponse).
"""
function executeQuery(
            connection::Neo4j.Connection,
            query::AS ) where {AS <: AbstractString}
  #
  loadtx = transaction(connection)
  cph = loadtx(query, submit=true)
  loadresult = commit(loadtx)
  return cph, loadresult
end
executeQuery(cg::CloudGraph, query::AS) where {AS <:AbstractString} = executeQuery(cg.neo4j.connection, query)

function getCloudVert(
            cg::CloudGraph,
            session::AbstractString,
            vsym::Symbol; bigdata::Bool=false )
  #
  warn("getCloudVert(cg, sess, sym) will be deprecated, use getCloudVert(cg, sess, sym=sym) instead.")
  # query = " and n.ready=$(ready) and n.label=$(vsym) "
  # query = reqbackendset ? query*" and n.backendset=$(backendset)" : query
  query = "match (n:$(session)) where n.label='$(vsym)' return id(n)"

  cph, = executeQuery(cg, query)
  # loadtx = transaction(cg.neo4j.connection)
  # cph = loadtx(query, submit=true)
  neoid = cph.results[1]["data"][1]["row"][1]
  CloudGraphs.get_vertex(cg, neoid, bigdata)
end

function getCloudVert(cgl::CloudGraph,
        session::AbstractString;
        exvid::VoidUnion{Int}=nothing,
        neoid::VoidUnion{Int}=nothing,
        sym::VoidUnion{Symbol}=nothing,
        bigdata=false  )
  #
  query = "match (n:$(session)) "

  if sym != nothing
    query = query*" where n.label='$(sym)' "
  elseif exvid != nothing
    query = query*" where n.exVertexId=$(exvid) "
  elseif neoid != nothing
    return CloudGraphs.get_vertex(cgl, neoid, bigdata)
  else
    @show sym, neoid,exvid
    error("Cannot list neighbors if no input reference is given")
  end
  query = query*" return id(n)"

  cph, = executeQuery(cgl.neo4j.connection, query)
  neoid = cph.results[1]["data"][1]["row"][1]
  CloudGraphs.get_vertex(cgl, neoid, bigdata)
end


function listAllVariables(cgl::CloudGraph, session::AbstractString)
  #
  query = "match (n:$(session)) where not (n:FACTOR) and exists(n.exVertexId) and n.ready=1 return n.label, n.exVertexId, id(n), labels(n)"
  cph, = executeQuery(cgl.neo4j.connection, query)

  dd = Dict{Symbol, Tuple{Int, Int, Vector{Symbol}}}()
  for metarows in cph.results[1]["data"]
    data = metarows["row"]
    lbls = setdiff(data[4],String[session])
    enl = (data[2], data[3], Symbol.(lbls))
    dd[Symbol(data[1])] = enl
  end
  return dd
end


function getCloudNeighbors(cgl::CloudGraph, cv::CloudVertex; needdata=false, returntype=false)
  neis = get_neighbors(cgl, cv, needdata=false)
end

"""
    $(SIGNATURES)

List neighbors to node in cgl::CloudGraph by returning Dict{Sym}=(exvid, neoid, Symbol[labels]), and can take
any of the three as input node identifier. Not specifying an identifier will result in all Variable nodes
being returned.
"""
function ls(cgl::CloudGraph, session::AbstractString;
      sym::VoidUnion{Symbol}=nothing,
      neoid::VoidUnion{Int64}=nothing,
      exvid::VoidUnion{Int64}=nothing  )
  #

  if sym == nothing && exvid == nothing && neoid == nothing
    # interrupt and just return all variable nodes
    return listAllVariables(cgl, session)
  end
  cv = getCloudVert(cgl, session, sym=sym, neoid=neoid, exvid=exvid, bigdata=false )

  neis = get_neighbors(cgl, cv, needdata=false)
  dd = Dict{Symbol, Tuple{Int, Int, Vector{Symbol}}}()
  for nei in neis
    lbls = setdiff(nei.labels, String[session])
    dd[Symbol(nei.properties["label"])] = (nei.exVertexId, nei.neo4jNodeId, Symbol.(lbls) )
  end
  return dd
end

function getfnctype(cvl::CloudGraphs.CloudVertex)
  vert = cloudVertex2ExVertex(cvl)
  return getfnctype(vert)
end

function initfg(;sessionname="NA",robotname="",cloudgraph=nothing)
  # fgl = RoME.initfg(sessionname=sessionname)
  fgl = IncrementalInference.emptyFactorGraph()
  fgl.sessionname = sessionname
  fgl.robotname = robotname
  fgl.cg = cloudgraph
  return fgl
end

function addCloudVert!{T <: AbstractString}(fgl::FactorGraph,
        exvert::Graphs.ExVertex;
        labels::Vector{T}=String[]  )
  #
  cv = CloudGraphs.exVertex2CloudVertex(exvert);
  cv.labels = labels
  CloudGraphs.add_vertex!(fgl.cg, cv);
  fgl.cgIDs[exvert.index] = cv.neo4jNodeId
  IncrementalInference.addGraphsVert!(fgl, exvert)
end

# Return Graphs.ExVertex type containing data according to id
function getExVertFromCloud(fgl::FactorGraph,
        fgid::Int64;
        bigdata::Bool=false  )
  #
  neoID = fgl.cgIDs[fgid]
  cvr = CloudGraphs.get_vertex(fgl.cg, neoID, false)
  CloudGraphs.cloudVertex2ExVertex(cvr)
end

function getExVertFromCloud(fgl::FactorGraph,
        lbl::Symbol;
        nt::Symbol=:var,
        bigdata::Bool=false  )
  # getExVertFromCloud(fgl, fgl.IDs[lbl], bigdata=bigdata)
  getExVertFromCloud(fgl, (nt==:var ? fgl.IDs[lbl] : fgl.fIDs[lbl]), bigdata=bigdata)
end

function updateFullCloudVertData!(fgl::FactorGraph,
        nv::Graphs.ExVertex;
        updateMAPest::Bool=false,
        bigdata::Bool=false  )
 #
  # TODO -- this get_vertex seems excessive, but we need the CloudVertex

  if(!haskey(fgl.cgIDs, nv.index))
      error("Cannot find $(nv.index) in $(fgl.cgIDs)...")
  end
  neoID = fgl.cgIDs[nv.index]
  # println("updateFullCloudVertData! -- trying to get $(neoID)")
  vert = CloudGraphs.get_vertex(fgl.cg, neoID, false)
  if typeof(getData(nv)) == VariableNodeData && updateMAPest
    mv = getKDEMax(getKDE(nv))
    nv.attributes["MAP_est"] = mv
    # @show nv.attributes["MAP_est"]
  end

  # TODO -- ignoring other properties
  vert.packed = getData(nv) #.attributes["data"]
  for pair in nv.attributes
    if pair[1] != "data"
      vert.properties[pair[1]] = pair[2]
    end
  end

  # also make sure our local copy is updated, need much better refactoring here
  fgl.stateless ? nothing : fgl.g.vertices[nv.index].attributes["data"] = nv.attributes["data"]

  CloudGraphs.update_vertex!(fgl.cg, vert, bigdata)
end

function makeAddCloudEdge!(fgl::FactorGraph, v1::Graphs.ExVertex, v2::Graphs.ExVertex)
  cv1 = CloudGraphs.get_vertex(fgl.cg, fgl.cgIDs[v1.index], false)
  cv2 = CloudGraphs.get_vertex(fgl.cg, fgl.cgIDs[v2.index], false)
  ce = CloudGraphs.CloudEdge(cv1, cv2, "DEPENDENCE");
  retrel = CloudGraphs.add_edge!(fgl.cg, ce);

  # TODO -- keep this edge id in function node data, must refactor
  push!(v2.attributes["data"].edgeIDs, retrel.id) # TODO -- not good way to do this
  updateFullCloudVertData!(fgl, v2)

  IncrementalInference.makeAddEdge!(fgl, v1, v2, saveedgeID=false)
  retrel.id
end


# TODO -- fetching of CloudVertex propably not required, make faster request to @GearsAD
function getCloudOutNeighbors(fgl::FactorGraph,
      exVertId::Int64;
      ready::Int=1,
      backendset::Int=1,
      needdata::Bool=false  )
  #
  # println("Looking for cloud out neighbors")
  cgid = fgl.cgIDs[exVertId]
  cv = CloudGraphs.get_vertex(fgl.cg, cgid, false)
  neighs = CloudGraphs.get_neighbors(fgl.cg, cv)
  neExV = Graphs.ExVertex[]
  for n in neighs
    cgn = CloudGraphs.cloudVertex2ExVertex(n)
    if (
       #cgn.attributes["ready"] == ready &&
       #cgn.attributes["backendset"] == backendset &&
       (!needdata || haskey(cgn.attributes, "data") )  )
      push!(neExV, cgn )
    end
  end
  return neExV
end

# return list of neighbors as Graphs.ExVertex type
function getCloudOutNeighbors(fgl::FactorGraph,
      vert::Graphs.ExVertex;
      ready::Int=1,
      backendset::Int=1,
      needdata::Bool=false  )
  # TODO -- test for ready and backendset here
  getCloudOutNeighbors(fgl, vert.index, ready=ready,backendset=backendset, needdata=needdata )
end


function getEdgeFromCloud(fgl::FactorGraph, id::Int64)
  println("getting id=$(id)")
  CloudGraphs.get_edge(fgl.cg, id)
end

function deleteCloudVertex!(fgl::FactorGraph, vert::Graphs.ExVertex)
  neoID = fgl.cgIDs[vert.index]
  cvr = CloudGraphs.get_vertex(fgl.cg, neoID, false)
  CloudGraphs.delete_vertex!(fgl.cg, cvr)
end

function deleteCloudEdge!(fgl::FactorGraph, edge::CloudEdge)
  CloudGraphs.delete_edge!(fgl.cg, edge)
end




function usecloudgraphsdatalayer!()
  IncrementalInference.setdatalayerAPI!(
    addvertex= addCloudVert!,
    getvertex= getExVertFromCloud,
    makeaddedge= makeAddCloudEdge!,
    getedge= getEdgeFromCloud,
    outneighbors= getCloudOutNeighbors,
    updatevertex= updateFullCloudVertData!,
    deletevertex= deleteCloudVertex!,
    deleteedge= deleteCloudEdge!,
    cgEnabled= true )
  nothing
end



# # setCloudDataLayerAPI!
# function setdatalayerAPI!(;
#       addvertex!::Function = addGraphsVert!,
#       getvertex::Function = getVertNode,
#       makeaddedge!::Function = makeAddEdge!,
#       getedge::Function = graphsGetEdge,
#       outneighbors::Function = graphsOutNeighbors,
#       updatevertex!::Function = updateFullVertData!,
#       updateedge!::Function = +,
#       deletevertex!::Function = graphsDeleteVertex!,
#       deleteedge!::Function = +,
#       cgEnabled::Function = false  )
#
#   dlapi.addvertex! = addvertex!
#   dlapi.getvertex = getvertex
#   dlapi.makeaddedge! = makeaddedge!
#   dlapi.getedge = getedge
#   dlapi.outneighbors = outneighbors
#   dlapi.updatevertex! = updatevertex!
#   dlapi.updateedge! = updateedge!
#   dlapi.deletevertex! = deletevertex!
#   dlapi.deleteedge! = deleteedge!
#   dlapi.cgEnabled = cgEnabled
#
#   # dlapi.addvertex! = addCloudVert!
#   # dlapi.getvertex = getExVertFromCloud
#   # dlapi.makeaddedge! = makeAddCloudEdge!
#   # dlapi.getedge = getEdgeFromCloud
#   # dlapi.updatevertex! = updateFullCloudVertData!
#   # dlapi.outneighbors = getCloudOutNeighbors
#   # dlapi.deletevertex! = deleteCloudVertex!
#   # dlapi.deleteedge! = deleteCloudEdge!
#   # dlapi.cgEnabled = true
#
#   println("Changed internal API calls to use outside calls.")
#   nothing
# end
# cgapi = DataLayerAPI(addCloudVert!,            # addvertex
#                      dlapi.getvertex,          # getvertex
#                      makeAddCloudEdge!,        # makeaddedge
#                      graphsGetEdge,           # getedge
#                      dlapi.outneighbors,       # outneighbors
#                      +, +, +, + )

function getpackedtype(typestring::AS) where {AS <: AbstractString}
  # println("Caesar.getpackedtype($(typestring))")
  eval(parse(typestring))() # TODO consider caching or better
end


# # register types of interest (Pose2, etc) in CloudGraphs
# # you can register new types at any time (Julia is dynamic)
# function registerGeneralVariableTypes!(cloudGraph::CloudGraph)
#   # Variable node
#   CloudGraphs.registerPackedType!(cloudGraph, VariableNodeData, PackedVariableNodeData, encodingConverter=VNDencoder, decodingConverter=VNDdecoder);
#   # factor nodes
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Obsv2}}, PackedFunctionNodeData{PackedObsv2}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Odo}}, PackedFunctionNodeData{PackedOdo}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{GenericMarginal}}, PackedFunctionNodeData{PackedGenericMarginal}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Ranged}}, PackedFunctionNodeData{PackedRanged}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#   # Pose2
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{PriorPose2}}, PackedFunctionNodeData{PackedPriorPose2}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Pose2Pose2}}, PackedFunctionNodeData{PackedPose2Pose2}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Pose2DPoint2DBearingRange{Distributions.Normal{Float64},Distributions.Normal{Float64}}}}, PackedFunctionNodeData{PackedPose2DPoint2DBearingRange}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Pose2DPoint2DRange}}, FunctionNodeData{Pose2DPoint2DRange}, encodingConverter=passTypeThrough, decodingConverter=passTypeThrough)
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{PriorPoint2D}}, PackedFunctionNodeData{PackedPriorPoint2D}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Point2DPoint2D}}, PackedFunctionNodeData{PackedPoint2DPoint2D}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{PriorPoint2DensityNH}}, PackedFunctionNodeData{PackedPriorPoint2DensityNH}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#   #acoustic types
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Pose2DPoint2DRangeDensity}}, PackedFunctionNodeData{PackedPose2DPoint2DRangeDensity}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Pose2DPoint2DBearingRangeDensity}}, PackedFunctionNodeData{PackedPose2DPoint2DBearingRangeDensity}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#   # Pose3 stuff
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{PriorPose3}}, PackedFunctionNodeData{PackedPriorPose3}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Pose3Pose3}}, PackedFunctionNodeData{PackedPose3Pose3}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Pose3Pose3NH}}, PackedFunctionNodeData{PackedPose3Pose3NH}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#   # partial constraints
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{PartialPriorRollPitchZ}}, PackedFunctionNodeData{PackedPartialPriorRollPitchZ}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{PartialPose3XYYaw}}, PackedFunctionNodeData{PackedPartialPose3XYYaw}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#   CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{PartialPose3XYYawNH}}, PackedFunctionNodeData{PackedPartialPose3XYYawNH}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
#
#   nothing
# end


# function should not be necessary, but fixes a minor bug following elimination algorithm
function removeGenericMarginals!(conn)
  loadtx = transaction(conn)
  query = "match (n)-[r]-() where n.packedType = 'IncrementalInference.FunctionNodeData{IncrementalInference.GenericMarginal}' detach delete n,r"
  cph = loadtx(query, submit=true)
  loadresult = commit(loadtx)
  # TODO -- can probably be made better, but should not be necessary in the first place
  loadtx = transaction(conn)
  query = "match (n) where n.packedType = 'IncrementalInference.FunctionNodeData{IncrementalInference.GenericMarginal}' detach delete n"
  cph = loadtx(query, submit=true)
  loadresult = commit(loadtx)
  nothing
end

"""
    $(SIGNATURES)

Get all Neo4j node IDs in current session.
"""
function getAllExVertexNeoIDs(conn::Neo4j.Connection;
        ready::Int=1,
        backendset::Int=1,
        sessionname::AS="",
        robotname::AS="",
        reqbackendset::Bool=true,
        reqready::Bool=true  ) where {AS <: AbstractString}
  #
  sn = length(sessionname) > 0 ? ":"*sessionname : ""
  rn = length(robotname) > 0 ? ":"*robotname : ""
  query = "match (n$(sn)$(rn)) where not n:SESSION and exists(n.exVertexId)"
  query = reqbackendset || reqready ? query*" and" : query
  query = reqready ? query*" n.ready=$(ready)" : query
  query = reqbackendset && reqready ? query*" and" : query
  query = reqbackendset ? query*" n.backendset=$(backendset)" : query
  query = query*" return n.exVertexId, id(n), n.label"

  cph, = executeQuery(conn, query)
  ret = Array{Tuple{Int64,Int64,Symbol},1}()

  # If no results, return empty array...
  if length(cph.results) > 0
      if length(cph.results[1]["data"]) > 0
          @showprogress 1 "Get ExVertex IDs..." for data in cph.results[1]["data"]
            exvid, neoid, sym = data["row"][1], data["row"][2], Symbol(data["row"][3])
            push!(ret, (exvid,neoid,sym)  )
          end
      end
  end

  return ret
end

# """
#     $(SIGNATURES)
#
# Build query to fetch sub graph and neighboring nodes.  For example:
# FAILS IN SOME CASES
# ```
# match (n0:Hackathon)-[:DEPENDENCE]-(n1:Hackathon)
# where n0.label IN ['x1', 'x2']
# with collect([
#   {id: id(n0)},
#   {id: id(n1)}
#   ]) as nodes
# unwind nodes as no
# unwind no as n
# match (m:Hackathon{ready:1,backendset:1})
# where id(m)=n.id
# return distinct id(m), m.label, m.exVertexId
# ```
# """
# function buildSubGraphIdsQueryOLD(;
#             lbls::Vector{AS}=String[""],
#             session::AS="",
#             robot::AS="",
#             label::AS="",
#             reqready::Bool=true,
#             ready::Int=1,
#             reqbackendset::Bool=true,
#             backendset::Int=1,
#             neighbors::Int=0  ) where {AS <: AbstractString}
#   #
#   sn = length(session) > 0 ? ":"*session : ""
#   rn = length(robot) > 0 ? ":"*robot : ""
#   lb = length(label) > 0 ? ":"*label : ""
#   query = "match (n0$(sn)$(rn)$(lb))"
#   for d in 1:neighbors
#     query *= "-[:DEPENDENCE]-(n$(d)$(sn)$(rn)$(lb))"
#   end
#   query *= " "
#   query *= "where n0.label IN ["
#   for lbl in lbls
#     query *= "'$(lbl)', "
#   end
#   query = chop(chop(query))*"]"
#   query *= "with collect(["
#   query *= "  {id: id(n0)},"
#   for d in 1:neighbors
#     query *= "  {id: id(n$(d))},"
#   end
#   query = chop(query)*"  ]) as nodes "
#   query *= "unwind nodes as no "
#   query *= "unwind no as n "
#   query *= "match (m$(sn)$(rn)$(lb)"
#   query *= reqready || reqbackendset ? "{" : ""
#   query *= reqready ? "ready:$(ready)" : ""
#   query *= reqready && reqbackendset ? ", " : ""
#   query *= reqbackendset ? "backendset:$(backendset)" : ""
#   query *= reqready || reqbackendset ? "}" : ""
#   query *= ") "
#   query *= "where id(m)=n.id "
#   query *= "return distinct m.exVertexId, id(m), m.label"
#   return query
# end

"""
    $(SIGNATURES)

Build query to fetch sub graph and neighboring nodes.  For example:
FAILS IN SOME CASES
```
match (n0:Hackathon:HexagonalDrive)
where n0.label IN ['x0']
with n0 as m
return m.exVertexId, id(m), m.label
UNION
match (n0:Hackathon:HexagonalDrive)-[:DEPENDENCE]-(n1:HexagonalDrive)
where n0.label IN ['x0']
with n1 as m
return m.exVertexId, id(m), m.label
UNION
match (n0:Hackathon:HexagonalDrive)-[:DEPENDENCE]-(n1:HexagonalDrive)-[:DEPENDENCE]-(n2:HexagonalDrive)
where n0.label IN ['x0']
with n2 as m
return m.exVertexId, id(m), m.label
```
"""
function buildSubGraphIdsQuery(;
            lbls::Vector{AS}=String[""],
            session::AS="",
            robot::AS="",
            label::AS="",
            reqready::Bool=true,
            ready::Int=1,
            reqbackendset::Bool=true,
            backendset::Int=1,
            neighbors::Int=0  ) where {AS <: AbstractString}
  #
  sn = length(session) > 0 ? ":"*session : ""
  rn = length(robot) > 0 ? ":"*robot : ""
  lb = length(label) > 0 ? ":"*label : ""

  query = ""
  for nei in 0:(neighbors)
    query *= "match (n0$(sn)$(rn)$(lb))"
    outerd = 0
    for d in 1:(nei)
      query *= "-[:DEPENDENCE]-(n$(d)$(sn)$(rn)$(lb))"
      outerd = d
    end
    query *= " "
    query *= "where n0.label IN ["
    for lbl in lbls
      query *= "'$(lbl)',"
    end
    query = chop(query)*"] "
    query *= "with n$(outerd) as m "
    query *= "return m.exVertexId, id(m), m.label"
    query *= nei != (neighbors) ? " UNION " : ""
  end
  return query
end
# qu = buildSubGraphIdsQuery(lbls=["x0";], session="HexagonalDrive", neighbors=3)
# @show qu

"""
    $(SIGNATURES)

Return array of tuples with ExVertex IDs and Neo4j IDs for vertices with label in session.
"""
function getLblExVertexNeoIDs(
        conn::Neo4j.Connection,
        lbls::Vector{AS};
        session::AS="",
        robot::AS="",
        label::AS="",
        reqready::Bool=true,
        ready::Int=1,
        backendset::Int=1,
        reqbackendset::Bool=true,
        neighbors::Int=0 ) where {AS <: AbstractString}
  #

  query = buildSubGraphIdsQuery(lbls=lbls, session=session, robot=robot, label=label, neighbors=neighbors, reqready=reqready, ready=ready, reqbackendset=reqbackendset, backendset=backendset)
  cph, = executeQuery(conn, query)

  ret = Array{Tuple{Int64,Int64,Symbol},1}()
  @showprogress 1 "Get ExVertex IDs..." for data in cph.results[1]["data"]
    exvid, neoid, vsym = data["row"][1], data["row"][2], Symbol(data["row"][3])
    push!(ret, (exvid,neoid,vsym)  )
  end
  return ret
end

"""
    $(SIGNATURES)

Return array of tuples with ExVertex IDs and Neo4j IDs for vertices with label in session.
"""
function getExVertexNeoIDs(
        conn::Neo4j.Connection;
        label::AS="",
        ready::Int=1,
        backendset::Int=1,
        session::AS="",
        reqbackendset::Bool=true  ) where {AS <: AbstractString}
  #
  sn = length(session) > 0 ? ":"*session : ""
  lb = length(label) > 0 ? ":"*label : ""
  query = "match (n$(sn)$(lb)) where n.ready=$(ready) and exists(n.exVertexId)"
  query = reqbackendset ? query*" and n.backendset=$(backendset)" : query
  query = query*" return n.exVertexId, id(n), n.label"

  cph, = executeQuery(conn, query)
  # loadtx = transaction(conn)
  # cph = loadtx(query, submit=true)

  ret = Array{Tuple{Int64,Int64,Symbol},1}()
  @showprogress 1 "Get ExVertex IDs..." for data in cph.results[1]["data"]
    exvid, neoid, vsym = data["row"][1], data["row"][2], Symbol(data["row"][3])
    push!(ret, (exvid,neoid,vsym)  )
  end
  return ret
end

"""
    $(SIGNATURES)

Return array of tuples with ExVertex IDs and Neo4j IDs for all poses.
"""
function getPoseExVertexNeoIDs(conn::Neo4j.Connection;
        ready::Int=1,
        backendset::Int=1,
        session::AS="",
        reqbackendset::Bool=true  ) where {AS <: AbstractString}
  #
  getPoseExVertexNeoIDs(conn,
          label="POSE",
          ready=ready,
          backendset=backendset,
          session=session,
          reqbackendset=reqbackendset  )
end


function checkandinsertedges!(fgl::FactorGraph, exvid::Int, nei::CloudVertex; ready::Int=1, backendset::Int=1)
  if nei.properties["ready"]==ready &&
     nei.properties["backendset"] == backendset &&
     haskey(fgl.g.vertices, nei.exVertexId)
    #&& nei.exVertexId <= length(fgl.g.vertices)
    alreadythere = false

    # TODO -- error point
    v2 = fgl.g.vertices[nei.exVertexId]
    for graphsnei in Graphs.out_neighbors(v2, fgl.g) # specifically want Graphs function
      # want to ignore if the edge was previously added from the other side, comparing to the out neighbors in the Graphs structure
      if graphsnei.index == exvid #graphsnei.index == nei.exVertexId
        alreadythere = true
        break;
      end
    end
    if !alreadythere
      # add the edge to graph
      v1 = fgl.g.vertices[exvid]
      makeAddEdge!(fgl, v1, v2, saveedgeID=false)
    end
  end
  nothing
end

function copyAllEdges!(fgl::FactorGraph, cverts::Dict{Int64, CloudVertex}, IDs::Array{Tuple{Int64,Int64, Symbol},1})
  # TODO -- major opportunity for streamlining and improving performance
  # do entire graph, one node at a time
  @showprogress 1 "Copy all edges..." for ids in IDs
    for nei in CloudGraphs.get_neighbors(fgl.cg, cverts[ids[2]], needdata=true)
      checkandinsertedges!(fgl, ids[1], nei, ready=1, backendset=1)
    end
  end
  nothing
end

function insertnodefromcv!(fgl::FactorGraph, cvert::CloudGraphs.CloudVertex)
  exvid = cvert.exVertexId
  neoid = cvert.neo4jNodeId
  exvert = cloudVertex2ExVertex(cvert)
  # TODO -- change to addNode
  Graphs.add_vertex!(fgl.g, exvert)
  fgl.id < exvert.index ? fgl.id = exvert.index : nothing
  fgl.cgIDs[exvid] = neoid
  if typeof(exvert.attributes["data"]) == VariableNodeData  # variable node
    fgl.IDs[Symbol(exvert.label)] = exvid
    push!(fgl.nodeIDs, exvid)
  else # function node
    fgl.fIDs[Symbol(exvert.label)] = exvid
    push!(fgl.factorIDs, exvid)
  end
  nothing
end

"""
    $(SIGNATURES)

Copy all variable and factor nodes from DB into fgl as listed in IDs vector.
"""
function copyAllNodes!(
            fgl::FactorGraph,
            cverts::Dict{Int64, CloudVertex},
            IDs::Array{Tuple{Int64,Int64, Symbol},1}  )
            # conn::Neo4j.Connection  ) # what the?
  #
  @showprogress 1 "Copy all nodes..." for ids in IDs
    cvert = CloudGraphs.get_vertex(fgl.cg, ids[2], false)
    cverts[ids[2]] = cvert
    insertnodefromcv!(fgl, cvert)
  end
  nothing
end

"""
    $(SIGNATURES)

Copy nodes into `fgl` from DB as listed in the `IDs` vector.
"""
function copyGraphNodesEdges!(
            fgl::FactorGraph,
            IDs::Array{Tuple{Int64,Int64, Symbol},1}  )
  #
  if length(IDs) > 0
    cverts = Dict{Int64, CloudVertex}()
    unsorted = Int64[]
    for ids in IDs
      push!(unsorted, ids[1])
    end
    perm = sortperm(unsorted)

    # get and add all the nodes
    sortedIDs = IDs[perm]
    copyAllNodes!(fgl, cverts, sortedIDs)

    # get and insert all edges
    copyAllEdges!(fgl, cverts, sortedIDs)
    return true
  else
    print(".")
    return false
  end
end

"""
    $(SIGNATURES)

Copy sub graph portion defined by, and including a depth of neighbors, from the lbls vector.
"""
function subLocalGraphCopy!(
            fgl::FactorGraph,
            lbls::Union{Vector{AS}, Vector{Symbol}};
            neighbors::Int=0,
            reqbackendset::Bool=true,
            reqready::Bool=true ) where {AS <: AbstractString}
  #
  warn("subGraphCopy! is a work in progress")
  conn = fgl.cg.neo4j.connection
  IDs = getLblExVertexNeoIDs(conn, string.(lbls), session=fgl.sessionname, robot=fgl.robotname, reqbackendset=reqbackendset, reqready=reqready, neighbors=neighbors )
  println("fullSubGraphCopy: $(length(IDs)) nodes in session $(fgl.sessionname) if reqbackendset=$reqbackendset and reqready=$reqready...")
  copyGraphNodesEdges!(fgl, IDs)
  nothing
end


"""
    $(SIGNATURES)

Fetch a full copy of the DB factor graph under fgl.sessionname.
"""
function fullLocalGraphCopy!(
            fgl::FactorGraph;
            reqbackendset::Bool=true,
            reqready::Bool=true  )
  #
  conn = fgl.cg.neo4j.connection
  IDs = getAllExVertexNeoIDs(conn, sessionname=fgl.sessionname, robotname=fgl.robotname, reqbackendset=reqbackendset, reqready=reqready)
  println("fullLocalGraphCopy: $(length(IDs)) nodes in robot=$(fgl.robotname), session $(fgl.sessionname) if reqbackendset=$reqbackendset and reqready=$reqready...")
  copyGraphNodesEdges!(fgl, IDs)
end

"""
    $(SIGNATURES)

Set all Neo4j nodes in this session ready = 1, warning function does not support new SynchronySDK data storage formats.
"""
function setDBAllReady!(
            conn::Neo4j.Connection,
            sessionname::AS) where {AS <: AbstractString}
  #
  warn("Obsolete setDBAllReady! function, see SynchronySDK for example ready function instead.")
  sn = length(sessionname) > 0 ? ":"*sessionname : ""
  query = "match (n$(sn)) set n.ready=1"
  cph, loadresult = executeQuery(conn, query)
  nothing
end

# TODO --this will only work with DB version, introduces a bug
function setDBAllReady!(fgl::FactorGraph)
  setDBAllReady!(fgl.cg.neo4j.connection, fgl.sessionname)
end


function setBackendWorkingSet!(
            conn::Neo4j.Connection,
            sessionname::AbstractString  )
  #
  sn = length(sessionname) > 0 ? ":"*sessionname : ""
  query = "match (n$(sn)) where not (n:NEWDATA) set n.backendset=1"

  cph, loadresult = executeQuery(conn, query)
  nothing
end

"""
    $(SIGNATURES)

Obtain Neo4j global database address and login credientials from STDIN, then insert and return in the addrdict colletion.
"""
function askneo4jcredentials!(;addrdict=Dict{AbstractString,AbstractString}() )
  need = ["neo4jHost";"neo4jPort";"neo4jUsername";"neo4jPassword";"session"]
  info("Please enter information for Neo4j DB:")
  for n in need
    info(n)
    str = readline(STDIN)
    addrdict[n] = str
    if length(str) > 0
      if str[end] == "\n"
        addrdict[n] = str[1:(end-1)]
      end
    end
  end
  return addrdict
end

"""
    $(SIGNATURES)

Obtain Mongo database address and login credientials from STDIN, then insert and return in the addrdict colletion.
"""
function askmongocredentials!(;addrdict=Dict{AbstractString,AbstractString}() )
  need = ["mongoHost";"mongoPort";"mongoUsername";"mongoPassword"]
  info("Please enter information for MongoDB:")
  for n in need
    info(n)
    n == "mongoHost" && haskey(addrdict, "neo4jHost") ? print(string("[",addrdict["neo4jHost"],"]: ")) : nothing
    str = readline(STDIN)
    addrdict[n] = str
    if length(str) > 0
      if str[end] == "\n"
        addrdict[n] = str[1:(end-1)]
      end
    end
  end
  addrdict["mongoIsUsingCredentials"] = false,
  if addrdict["mongoHost"] == "" && haskey(addrdict, "neo4jHost")
    addrdict["mongoHost"] = addrdict["neo4jHost"]
  elseif addrdict["mongoHost"] != ""
    nothing
  else
    error("Don't know how to get to MongoDB address.")
  end
  return addrdict
end


"""
    $(SIGNATURES)

Obtain database addresses and login credientials from STDIN, as well as a few case dependent options.
"""
function consoleaskuserfordb(;nparticles=false, drawdepth=false, clearslamindb=false, multisession=false, drawedges=false)
  addrdict = Dict{AbstractString, Any}()  #Union{AbstractString, Vector{String}}
  askneo4jcredentials!(addrdict=addrdict)
  askmongocredentials!(addrdict=addrdict)
  need = String[]
  !nparticles ? nothing : push!(need, "num particles")
  !drawdepth ? nothing : push!(need, "draw depth")
  !clearslamindb ? nothing : push!(need, "clearslamindb")
  !multisession ? nothing : push!(need, "multisession")
  !drawedges ? nothing : push!(need, "draw edges")


  info("Please also enter information for:")
  for n in need
    info(n)
    n == "draw depth" ? print("[y]/n: ") : nothing
    n == "draw edges" ? print("[y]/n: ") : nothing
    n == "num particles" ? print("[100]: ") : nothing
    n == "clearslamindb" ? print("yes/[no]: ") : nothing
    n == "multisession" ? print("comma separated list session names/[n]: ") : nothing
    str = readline(STDIN)
    addrdict[n] = str
    if length(str) > 0
      if str[end] == "\n"
        addrdict[n] = str[1:(end-1)]
      end
    end
  end
  if drawdepth
    addrdict["draw depth"] = addrdict["draw depth"]=="" || addrdict["draw depth"]=="y" || addrdict["draw depth"]=="yes" ? "y" : "n"
  end
  if drawdepth
    addrdict["draw edges"] = addrdict["draw edges"]=="" || addrdict["draw edges"]=="y" || addrdict["draw edges"]=="yes" ? "y" : "n"
  end
  if nparticles
    addrdict["num particles"] = addrdict["num particles"]!="" ? addrdict["num particles"] : "100"
  end
  if clearslamindb
    addrdict["clearslamindb"] = addrdict["clearslamindb"]=="" || addrdict["clearslamindb"]=="n" || addrdict["clearslamindb"]=="no" ? "n" : addrdict["clearslamindb"]
  end
  if multisession
    addrdict["multisession"] = strip.(Vector{String}(split(addrdict["multisession"],',')))
  end
  return addrdict
end


"""
    $(SIGNATURES)

Connect to databases via network according to addrdict, or ask user for credentials and return
active cloudGraph object, as well as addrdict.
"""
function standardcloudgraphsetup(;addrdict=nothing,
            nparticles::Bool=false,
            drawdepth::Bool=false,
            drawedges::Bool=false,
            clearslamindb::Bool=false,
            multisession::Bool=false  )
  #
  if addrdict == nothing
    addrdict = consoleaskuserfordb(nparticles=nparticles, drawdepth=drawdepth, clearslamindb=clearslamindb, multisession=multisession, drawedges=drawedges)
  end

  warn("Not considering field: addrdict[\"mongoIsUsingCredentials\"]")
  # Connect to database
  # addrdict["mongoIsUsingCredentials"]
  configuration = CloudGraphs.CloudGraphConfiguration(
                             addrdict["neo4jHost"], parse(Int, addrdict["neo4jPort"]), addrdict["neo4jUsername"], addrdict["neo4jPassword"],
                             addrdict["mongoHost"], parse(Int, addrdict["mongoPort"]), false, addrdict["mongoUsername"], addrdict["mongoPassword"]);
  cloudGraph = connect(configuration, IncrementalInference.encodePackedType, Caesar.getpackedtype, IncrementalInference.decodePackedType);
  # conn = cloudGraph.neo4j.connection
  # register types of interest in CloudGraphs
  # registerGeneralVariableTypes!(cloudGraph) # TESTING -- using dispatch instead
  Caesar.usecloudgraphsdatalayer!()

  return cloudGraph, addrdict
end

"""
    $(SIGNATURES)

Walk through vertex bigDataElements and return the last matching description.
"""
function getBigDataElement(vertex::CloudVertex, description::AbstractString)
  bde = nothing
  for bDE in vertex.bigData.dataElements
    if bDE.description == description
      bde = bDE
    end
  end
  return bde
end

"""
    $(SIGNATURES)

Return true if vertex has bigDataElements with matching description.
"""
function hasBigDataElement(vertex::CloudVertex, description::AbstractString)
  for bDE in vertex.bigData.dataElements
    if bDE.description == description
      return true
    end
  end
  return false
end

"""
    $(SIGNATURES)

Append big data element into current blob store and update associated global
vertex information.
"""
function appendvertbigdata!(cloudGraph::CloudGraph,
        cv::CloudVertex,
        description,
        data::Vector{UInt8}  )
  #
  bd = CloudGraphs.read_BigData!(cloudGraph, cv)
  bdei = CloudGraphs.BigDataElement(description, data)
  push!(cv.bigData.dataElements, bdei);
  CloudGraphs.save_BigData!(cloudGraph, cv)
end

"""
    $(SIGNATURES)

Append big data element into current blob store and update associated global
vertex information.
"""
function appendvertbigdata!(fgl::FactorGraph,
      vert::Graphs.ExVertex,
      description::AbstractString,
      data::Vector{UInt8}  )
  #
  # TODO -- improve get/fetch vertex abstraction
  cvid = fgl.cgIDs[vert.index]
  cv = CloudGraphs.get_vertex(fgl.cg, cvid, true)
  appendvertbigdata!(fgl.cg, cv, description, data)
end


"""
    appendvertbigdata!(fg, sym, descr, data)

Append big data element into current blob store using parent appendvertbigdata!,
but here specified by symbol of variable node in the FactorGraph. Note the
default data layer api definition. User must define dlapi to refetching the
 vertex from the data layer. localapi avoids repeated network database fetches.
"""
function appendvertbigdata!(fgl::FactorGraph,
      sym::Symbol,
      description::AbstractString,
      data;
      api=IncrementalInference.localapi  )
  #
  appendvertbigdata!(fgl,
        getVert(fgl, sym, api=api),
        description,
        data  )
end


"""
    fetchsubgraph!(::FactorGraph, ::Vector{CloudVertex}, numneighbors::Int=0)

Fetch and insert list of CloudVertices into FactorGraph object, up to neighbor depth.
"""
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
      insertnodefromcv!(fgl, cv)

      # recursive call on neighbors here
      neicvs = CloudGraphs.get_neighbors(fgl.cg, cv, needdata=true)
      fetchsubgraph!(fgl, neicvs; numneighbors=numneighbors-1 )
      # add edges associated with the neighbors

      if numneighbors-1 >= 0
        for cvn in neicvs
          checkandinsertedges!(fgl, cv.exVertexId, cvn, ready=1, backendset=1)
          # makeAddEdge!(fgl, fgl.g.vertices[cv.exVertexId], fgl.g.vertices[cvn.exVertexId], saveedgeID=false)
        end
      end
    end

  end
end

"""
    fetchsubgraph!(::FactorGraph, ::Vector{Int}, numneighbors::Int=0)

Fetch and insert list of Neo4j IDs into FactorGraph object, up to neighbor depth.
"""
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

"""
    getVertNeoIDs!(::CloudGraph, res::Dict{Symbol, Int}; session::AbstractString="NA")

Insert into and return dict `res` with Neo4j IDs of ExVertex labels as stored per session in Neo4j database.
"""
function getVertNeoIDs!(cloudGraph::CloudGraph, res::Dict{Symbol, Int}; session::AbstractString="NA")
  loadtx = transaction(cloudGraph.neo4j.connection)
  syms = collect(keys(res))
  query = "match (n:$(session)) where "
  for i in 1:length(syms)
    sym = syms[i]
    query =query*"n.label='$(sym)' "
    if i < length(syms)
      query =query*"or "
    end
  end
  query = query*"return id(n), n.label"
  cph = loadtx(query, submit=true)
  for qr in cph.results[1]["data"]
    res[Symbol(qr["row"][2])] = qr["row"][1]
  end
  # Symbol(cph.results[1]["data"][1]["row"][2]) == sym ? nothing : error("Neo4j query not returning $(sym)")
  return res
end

"""
    removeNeo4jID(cg::CloudGraph, neoid=-1)

Remove node from Neo4j according to Neo4j Node ID. Big data elements that may be associated with this
node are not removed.
"""
function removeNeo4jID(cg::CloudGraph; neoid::Int=-1)
  neoid > 0 ? nothing : error("Can't delete negative neoid=$(neoid).")
  loadtx = transaction(cg.neo4j.connection)
  query =  "match (n) where id(n)=$(neoid) detach delete n return count(n)"
  cph = loadtx(query, submit=true)
  commit(loadtx)
  cnt = Int(cph.results[1]["data"][1]["row"][1])
  cnt == 1 ? nothing : error("Did not delete just one entry after running query = $(query)")
  nothing
end



"""
    getfirstpose(cg::CloudGraph, session::AbstractString)

Return Tuple{Symbol, Int} of first pose symbol and Neo4j node ID.
"""
function getfirstpose(cg::CloudGraph, session::AbstractString)
  query = "match (n:$(session):POSE) with n.label as nlbl, n.exVertexId as exvid, id(n) as neoid order by exvid asc limit 1 return nlbl, neoid"
  cph, = executeQuery(cg, query)
  # loadtx = transaction(cg.neo4j.connection)
  # cph = loadtx(query, submit=true)
  Symbol(cph.results[1]["data"][1]["row"][1]), cph.results[1]["data"][1]["row"][2]
end
getFirstPose(cg::CloudGraph, session::AbstractString) = getfirstpose(cg, session)

"""
    getLastPose(cg::CloudGraph, session::AbstractString)

Return Tuple{Symbol, Int} of first pose symbol and Neo4j node ID.
"""
function getLastPose(cg::CloudGraph, session::AbstractString)
  query = "match (n:$(session):POSE) with n.label as nlbl, n.exVertexId as exvid, id(n) as neoid order by exvid desc limit 1 return nlbl, neoid"
  cph, = executeQuery(cg, query)
  # loadtx = transaction(cg.neo4j.connection)
  # cph = loadtx(query, submit=true)
  Symbol(cph.results[1]["data"][1]["row"][1]), cph.results[1]["data"][1]["row"][2]
end


"""
    insertrobotdatafirstpose!(cg::CloudGraph, session::AbstractString, robotdict::Dict)

Saves robotdict via JSON to first pose in a SESSION in the database. Used for
storing general robot specific data in easily accessible manner. Can fetch later
retrieve same dict with counterpart `fetchrobotdatafirstpose` function.
"""
function insertrobotdatafirstpose!(cg::CloudGraph, session::AbstractString, robotdict::Dict)
  vsym, neoid = getfirstpose(cg, session)
  cv = CloudGraphs.get_vertex(cg, neoid, true)
  appendvertbigdata!(cg, cv, "robot_description", json(robotdict).data  )
end

function tryunpackalltypes!(resp::Dict)
  for (k,v) in resp
    tv = typeof(v)
    if tv == Vector{Any}
      ttv = typeof(v[1])
      if ttv == Vector{Any}
        try resp[k] = hcat(v...) catch end
      else
        try resp[k] = Vector{ttv}(v) catch end
      end
    end
  end
  nothing
end

"""
    fetchrobotdatafirstpose(cg::CloudGraph, session::AbstractString)

Return dict of JSON parsed "robot_description" field as was inserted by counterpart
`insertrobotdatafirstpose!` function. Used for storing general robot specific data
in easily accessible manner.
"""
function fetchrobotdatafirstpose(cg::CloudGraph, session::AbstractString)
  vsym, neoid = getfirstpose(cg, session)
  cv = CloudGraphs.get_vertex(cg, neoid, true)
  bde = Caesar.getBigDataElement(cv, "robot_description")
  resp = JSON.parse(String(take!(IOBuffer(bde.data))))
  tryunpackalltypes!(resp)
  return resp
end


"""
    getRangeKDEMax2D(cgl::CloudGraph, session::AbstractString, vsym1::Symbol, vsym2::Symbol)

Calculate the cartesian distange between two vertices in the graph, by session and symbol names,
and by maximum belief point.
"""
function getRangeKDEMax2D(cgl::CloudGraph, session::AbstractString, vsym1::Symbol, vsym2::Symbol)
  # get the relavent neo4j ids
  syms = Dict(vsym1=>0, vsym2 => 0)
  getVertNeoIDs!(cgl, syms, session=session)

  # build a local subgraph
  sfg = initfg(cloudgraph=cgl, sessionname=session)
  fetchsubgraph!(sfg, collect(values(syms)), numneighbors=1 )

  # calculate distances on local subgraph
  getRangeKDEMax2D(sfg, vsym1, vsym2)
end


"""
    db2jld(cgl::CloudGraph, session::AbstractString, filename::AbstractString)

Fetch and save a FactorGraph session to a jld, using CloudGraph object and session definition.
"""
function db2jld(cgl::CloudGraph, session::AbstractString, filename::AbstractString)
  fg = Caesar.initfg(sessionname=session, cloudgraph=cgl)
  fullLocalGraphCopy!(fg)
  savejld(fg, file=filename)
  return fg
end

"""
    db2jld(filename::AbstractString; addrdict::VoidUnion{Dict{AbstractString, AbstractString}}=nothing )

Fetch and save a FactorGraph session to a jld, using or asking STDIN for credentials in the addrdict field.
"""
function db2jld(filename::AbstractString; addrdict::VoidUnion{Dict{AbstractString, AbstractString}}=nothing )
  cg, cr = standardcloudgraphsetup(addrdict=addrdict)
  db2jld(cg, cr["session"], filename)
end


function deleteServerSession!(cloudGraph::CloudGraph, session::AbstractString)
  query =  "match (n:$(session))
            detach delete n
            return count(n)"
  return executeQuery(cloudGraph.neo4j.connection, query)
end

# function syncmongos()
#
# end

  #
