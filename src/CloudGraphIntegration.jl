# integration code for database usage via CloudGraphs.jl

function initfg(;sessionname="NA",cloudgraph=nothing)
  fgl = RoME.initfg(sessionname=sessionname)
  fgl.cg = cloudgraph
  return fgl
end

function addCloudVert!(fgl::FactorGraph, exvert::Graphs.ExVertex;
    labels=String[])
  # if typeof(getData(exvert).fnc)==GenericMarginal
  #   error("Should not be here")
  # end
  cv = CloudGraphs.exVertex2CloudVertex(exvert);
  cv.labels = labels
  CloudGraphs.add_vertex!(fgl.cg, cv);
  fgl.cgIDs[exvert.index] = cv.neo4jNodeId
  IncrementalInference.addGraphsVert!(fgl, exvert)
end

# Return Graphs.ExVertex type containing data according to id
function getExVertFromCloud(fgl::FactorGraph, fgid::Int64; bigdata::Bool=false)
  neoID = fgl.cgIDs[fgid]
  cvr = CloudGraphs.get_vertex(fgl.cg, neoID, false)
  CloudGraphs.cloudVertex2ExVertex(cvr)
end

function getExVertFromCloud(fgl::FactorGraph, lbl::Symbol; bigdata::Bool=false)
  getExVertFromCloud(fgl, fgl.IDs[lbl], bigdata=bigdata)
end

function updateFullCloudVertData!(fgl::FactorGraph,
    nv::Graphs.ExVertex;
    updateMAPest::Bool=false )

  # TODO -- this get_vertex seems excessive, but we need the CloudVertex
  neoID = fgl.cgIDs[nv.index]
  # println("updateFullCloudVertData! -- trying to get $(neoID)")
  vert = CloudGraphs.get_vertex(fgl.cg, neoID, false)

  if typeof(getData(nv)) == VariableNodeData && updateMAPest
    mv = getKDEMax(getKDE(nv))
    nv.attributes["MAP_est"] = mv
    # @show nv.attributes["MAP_est"]
  end

  # TODO -- ignoring other properties
  vert.packed = nv.attributes["data"]
  for pair in nv.attributes
    if pair[1] != "data"
      vert.properties[pair[1]] = pair[2]
    end
  end

  # also make sure our local copy is updated, need much better refactoring here
  fgl.g.vertices[nv.index].attributes["data"] = nv.attributes["data"]

  CloudGraphs.update_vertex!(fgl.cg, vert)
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
function getCloudOutNeighbors(fgl::FactorGraph, exVertId::Int64; ready::Int=1,backendset::Int=1)
  cgid = fgl.cgIDs[exVertId]
  cv = CloudGraphs.get_vertex(fgl.cg, cgid, false)
  neighs = CloudGraphs.get_neighbors(fgl.cg, cv)
  neExV = Graphs.ExVertex[]
  for n in neighs
    cgn = CloudGraphs.cloudVertex2ExVertex(n)
    if cgn.attributes["ready"] == ready && cgn.attributes["backendset"] == backendset
      push!(neExV, cgn )
    end
  end
  return neExV
end

# return list of neighbors as Graphs.ExVertex type
function getCloudOutNeighbors(fgl::FactorGraph, vert::Graphs.ExVertex; ready::Int=1,backendset::Int=1)
  # TODO -- test for ready and backendset here
  getCloudOutNeighbors(fgl, vert.index, ready=ready,backendset=backendset)
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



# register types of interest (Pose2, etc) in CloudGraphs
# you can register new types at any time (Julia is dynamic)
function registerGeneralVariableTypes!(cloudGraph::CloudGraph)
  # 1D early development types
  CloudGraphs.registerPackedType!(cloudGraph, VariableNodeData, PackedVariableNodeData, encodingConverter=VNDencoder, decodingConverter=VNDdecoder);
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Obsv2}}, PackedFunctionNodeData{PackedObsv2}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Odo}}, PackedFunctionNodeData{PackedOdo}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{GenericMarginal}}, PackedFunctionNodeData{PackedGenericMarginal}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Ranged}}, PackedFunctionNodeData{PackedRanged}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
  # Pose2
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{PriorPose2}}, PackedFunctionNodeData{PackedPriorPose2}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Pose2Pose2}}, PackedFunctionNodeData{PackedPose2Pose2}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Pose2DPoint2DBearingRange}}, PackedFunctionNodeData{PackedPose2DPoint2DBearingRange}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Pose2DPoint2DRange}}, FunctionNodeData{Pose2DPoint2DRange}, encodingConverter=passTypeThrough, decodingConverter=passTypeThrough)
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{PriorPoint2D}}, PackedFunctionNodeData{PackedPriorPoint2D}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
  # TODO -- Pose3 stuff
  nothing
end


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


function getAllExVertexNeoIDs(conn; ready::Int=1, backendset::Int=1, sessionname::AbstractString="")

  loadtx = transaction(conn)
  sn = length(sessionname) > 0 ? ":"*sessionname : ""
  query = "match (n$(sn)) where n.ready=$(ready) and n.backendset=$(backendset) return n"
  # query = "match (n) where n.ready=1 and n.backendset=1 and not(n.packedType = 'IncrementalInference.FunctionNodeData{IncrementalInference.GenericMarginal}') return n"
  cph = loadtx(query, submit=true)
  ret = Array{Tuple{Int64,Int64},1}()

  for data in cph.results[1]["data"]
    metadata = data["meta"][1]
    rowdata = data["row"][1]
    push!(ret, (rowdata["exVertexId"],metadata["id"])  )
  end
  return ret
end

# return array of tuples with exvertex and neo4j IDs for all poses
function getPoseExVertexNeoIDs(conn; ready::Int=1, backendset::Int=1, sessionname::AbstractString="")
  # TODO -- in query we can use return n.exVertexId, n.neo4jNodeId
  # TODO -- in query we can use n:POSE rather than length(n.MAP_est)=3

  loadtx = transaction(conn)
  # query = "match (n:$(sessionname)) where n.ready=$(ready) and n.backendset=$(backendset) and n.packedType = 'IncrementalInference.PackedVariableNodeData' and length(n.MAP_est)=3 return n"
  sn = length(sessionname) > 0 ? ":"*sessionname : ""
  query = "match (n$(sn)) where n:POSE and n.ready=$(ready) and n.backendset=$(backendset) return n"
  cph = loadtx(query, submit=true)
  ret = Array{Tuple{Int64,Int64},1}()

  for data in cph.results[1]["data"]
    metadata = data["meta"][1]
    rowdata = data["row"][1]
    push!(ret, (rowdata["exVertexId"],metadata["id"])  )
  end
  return ret
end

# function getDBAdjMatrix()
#
# end

function copyAllEdges!(fgl::FactorGraph, cverts::Dict{Int64, CloudVertex}, IDs::Array{Tuple{Int64,Int64},1})
  # do entire graph, one node at a time
  for ids in IDs
    # look at neighbors of this node
    for nei in CloudGraphs.get_neighbors(fgl.cg, cverts[ids[2]])
      # allow = true
      # if haskey(nei.properties, "packedType")
      #   if nei.properties["packedType"] == "IncrementalInference.FunctionNodeData{IncrementalInference.GenericMarginal}"
      #     allow = false
      #   end
      # end
      # TODO -- remove last test, since only works for array
      if nei.properties["ready"]==1 && nei.properties["backendset"] == 1 && nei.exVertexId <= length(fgl.g.vertices)
          alreadythere = false
          # TODO -- error point
          v2 = fgl.g.vertices[nei.exVertexId]
          for graphsnei in Graphs.out_neighbors(v2, fgl.g) # specifically want Graphs function
            # want to ignore if the edge was previously added from the other side, comparing to the out neighbors in the Graphs structure
            if graphsnei.index == ids[1] #graphsnei.index == nei.exVertexId
              alreadythere = true
              break;
            end
          end
          if !alreadythere
            # add the edge to graph
            v1 = fgl.g.vertices[ids[1]]
            makeAddEdge!(fgl, v1, v2, saveedgeID=false)
          end
      end
    end
  end
  nothing
end

function copyAllNodes!(fgl::FactorGraph, cverts::Dict{Int64, CloudVertex}, IDs::Array{Tuple{Int64,Int64},1}, conn)
  for ids in IDs
    cvert = CloudGraphs.get_vertex(fgl.cg, ids[2], false)
    cverts[ids[2]] = cvert
    exvert = cloudVertex2ExVertex(cvert)
    Graphs.add_vertex!(fgl.g, exvert)
    fgl.id < exvert.index ? fgl.id = exvert.index : nothing
    fgl.cgIDs[ids[1]] = ids[2]
    if typeof(exvert.attributes["data"]) == VariableNodeData  # variable node
      fgl.IDs[Symbol(exvert.label)] = ids[1]
      push!(fgl.nodeIDs, ids[1])
    else # function node
      fgl.fIDs[Symbol(exvert.label)] = ids[1]
      push!(fgl.factorIDs, ids[1])
    end
  end
  nothing
end

function fullLocalGraphCopy!(fgl::FactorGraph, conn)

  IDs = getAllExVertexNeoIDs(conn, sessionname=fgl.sessionname)
  if length(IDs) > 1
    cverts = Dict{Int64, CloudVertex}()
    unsorted = Int64[]
    # TODO ensure this is row is sorted
    for ids in IDs push!(unsorted, ids[1]) end
    perm = sortperm(unsorted)
    # testlist = deepcopy(unsorted)
    # if testlist != sort(unsorted)
    #   # TODO -- maybe not required, but being safe for now
    #   error("Must be sorted list for elimination...")
    # end

    # get and add all the nodes
    sortedIDs = IDs[perm]
    copyAllNodes!(fgl, cverts, sortedIDs, conn)

    # get and insert all edges
    copyAllEdges!(fgl, cverts, sortedIDs)
    return true
  else
    print(".")
    return false
  end
end

function setDBAllReady!(conn, sessionname)
  loadtx = transaction(conn)
  sn = length(sessionname) > 0 ? ":"*sessionname : ""
  query = "match (n$(sn)) where n.ready=0 set n.ready=1"
  cph = loadtx(query, submit=true)
  loadresult = commit(loadtx)
  nothing
end

# TODO --this will only work with DB version, introduces a bug
function setDBAllReady!(fgl::FactorGraph)
  setDBAllReady!(fgl.cg.neo4j.connection, fgl.sessionname)
end


function setBackendWorkingSet!(conn, sessionname::AbstractString)
  loadtx = transaction(conn)
  sn = length(sessionname) > 0 ? ":"*sessionname : ""
  query = "match (n$(sn)) set n.backendset=1"
  cph = loadtx(query, submit=true)
  loadresult = commit(loadtx)
  nothing
end






  #
