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
  # Variable node
  CloudGraphs.registerPackedType!(cloudGraph, VariableNodeData, PackedVariableNodeData, encodingConverter=VNDencoder, decodingConverter=VNDdecoder);
  # factor nodes
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Obsv2}}, PackedFunctionNodeData{PackedObsv2}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Odo}}, PackedFunctionNodeData{PackedOdo}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{GenericMarginal}}, PackedFunctionNodeData{PackedGenericMarginal}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Ranged}}, PackedFunctionNodeData{PackedRanged}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
  # Pose2
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{PriorPose2}}, PackedFunctionNodeData{PackedPriorPose2}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Pose2Pose2}}, PackedFunctionNodeData{PackedPose2Pose2}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Pose2DPoint2DBearingRange{Distributions.Normal,Distributions.Normal}}}, PackedFunctionNodeData{PackedPose2DPoint2DBearingRange}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Pose2DPoint2DRange}}, FunctionNodeData{Pose2DPoint2DRange}, encodingConverter=passTypeThrough, decodingConverter=passTypeThrough)
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{PriorPoint2D}}, PackedFunctionNodeData{PackedPriorPoint2D}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
  #acoustic types
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Pose2DPoint2DRangeDensity}}, PackedFunctionNodeData{PackedPose2DPoint2DRangeDensity}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
  CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{GenericWrapParam{Pose2DPoint2DBearingRangeDensity}}, PackedFunctionNodeData{PackedPose2DPoint2DBearingRangeDensity}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
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


function getAllExVertexNeoIDs(conn;
        ready::Int=1,
        backendset::Int=1,
        sessionname::AbstractString="",
        reqbackendset::Bool=true  )
  #
  loadtx = transaction(conn)
  sn = length(sessionname) > 0 ? ":"*sessionname : ""
  # query = "match (n$(sn)) where n.ready=$(ready) and n.backendset=$(backendset) return n"
  query = "match (n$(sn)) where n.ready=$(ready)"
  query = reqbackendset ? query*" and n.backendset=$(backendset)" : query
  query = query*" return n"

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
function getPoseExVertexNeoIDs(conn;
        ready::Int=1,
        backendset::Int=1,
        sessionname::AbstractString="",
        reqbackendset::Bool=true  )
  #
  # TODO -- in query we can use return n.exVertexId, n.neo4jNodeId
  # TODO -- in query we can use n:POSE rather than length(n.MAP_est)=3

  loadtx = transaction(conn)
  # query = "match (n:$(sessionname)) where n.ready=$(ready) and n.backendset=$(backendset) and n.packedType = 'IncrementalInference.PackedVariableNodeData' and length(n.MAP_est)=3 return n"
  sn = length(sessionname) > 0 ? ":"*sessionname : ""
  query = "match (n$(sn)) where n:POSE and n.ready=$(ready)"
  query = reqbackendset ? query*" and n.backendset=$(backendset)" : query
  query = query*" return n"
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
      if !haskey(fgl.g.vertices, nei.exVertexId)
        warn("skip neighbor if not in the subgraph segment of interest, exVertexId=$(nei.exVertexId)")
        continue;
      end
      # allow = true
      # if haskey(nei.properties, "packedType")
      #   if nei.properties["packedType"] == "IncrementalInference.FunctionNodeData{IncrementalInference.GenericMarginal}"
      #     allow = false
      #   end
      # end
      # TODO -- remove last test, since only works for array
      if nei.properties["ready"]==1 && nei.properties["backendset"] == 1 #&& nei.exVertexId <= length(fgl.g.vertices)
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

function fullLocalGraphCopy!(fgl::FactorGraph, conn; reqbackendset::Bool=true)

  IDs = getAllExVertexNeoIDs(conn, sessionname=fgl.sessionname, reqbackendset=reqbackendset)
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
  query = "match (n$(sn)) set n.ready=1"
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



"""
    getnewvertdict(conn, session)

Return a dictionary with frtend and mongo_keys json string information for :NEWDATA
elements in Neo4j database.
"""
function getnewvertdict(conn, session::AbstractString)

  loadtx = transaction(conn)
  query = "match (n:$(session))-[:DEPENDENCE]-(f:NEWDATA:$(session):FACTOR) where n.ready=1 or f.ready=1 return distinct n, f"
  cph = loadtx(query, submit=true)
  # loadresult = commit(loadtx)
  # @show cph.results[1]

  newvertdict = SortedDict{Int, Dict{Symbol, Dict{AbstractString,Any}}, Base.Order.ForwardOrdering}()
  # mongokeydict = Dict{Int, Dict{AbstractString,Any}}()

  for val in cph.results[1]["data"]
    i = 0
    for elem in val["meta"]
      # @show elem["type"]    # @show rdict["type"]
      i+=1
      newvertdict[elem["id"]] = Dict{Symbol, Dict{AbstractString,Any}}()
      for (k,nv) in val["row"][i]
        if Symbol(k) == :frtend
          newvertdict[elem["id"]][Symbol(k)] = JSON.parse(nv)
        else
          newvertdict[elem["id"]][Symbol(k)] = Dict{AbstractString, Any}("val"=> nv)
        end
      end
      # rdict = JSON.parse(val["row"][i]["frtend"])
      # newvertdict[elem["id"]][:frtend] = rdict
      # if haskey(val["row"][i], "mongo_keys")
      #   # @show val["row"][i]["mongo_keys"]
      #   newvertdict[elem["id"]][:mongokeys] = JSON.parse(val["row"][i]["mongo_keys"])
      # end
      # # if uppercase(rdict["type"])=="POSE" || uppercase(rdict["type"])=="FACTOR"
      #   # npsym = Symbol(string("x",parse(Int, rdict["userid"])+1)) # TODO -- fix :x0 requirement
    end
    # println()
  end

  return newvertdict
end




function insertValuesCloudVert!(fgl::FactorGraph, neoNodeId::Int, elem, uidl, v::Graphs.ExVertex; labels=String[])
  #
  # addNode!(fgl, nlbsym, 0.1*randn(3,N), 0.01*eye(3), N=N, ready=0, uid=uidl,api=localapi)

  # v = getVert(fgl, nlbsym, api=localapi)
  # warn("deleting frntend values")
  # delete!(v.attributes,"frntend")

  # v.attributes["frtend"] = JSON.json(elem[:frtend])
  # if haskey(elem, :mongokeys)
  #   v.attributes["mongo_keys"] = JSON.json(elem[:mongokeys])
  # end
  for (k,va) in elem
    if k != :frtend
      v.attributes[string(k)] = va["val"]
    else
      v.attributes[string(k)] = JSON.json(va)
    end
  end

  fgl.cgIDs[uidl] = neoNodeId

  cv = exVertex2CloudVertex( v )
  cv.neo4jNodeId = neoNodeId
  cv.neo4jNode = Neo4j.getnode(fgl.cg.neo4j.graph, neoNodeId)
  cv.isValidNeoNodeId = true
  cv.labels=labels
  #
  CloudGraphs.update_vertex!(fgl.cg, cv)
  # TODO -- this function refactoring and better integration here
  # function updateFullCloudVertData!( fgl::FactorGraph,
  #     nv::Graphs.ExVertex; updateMAPest=false )
  nothing
end


function recoverConstraintType(elem; N::Int=200)
  lkl = split(elem["lklh"], ' ')
  if lkl[1]=="PR2"
    msm = split(elem["meas"], ' ')
    cov = zeros(3,3)
    cov[1,2], cov[1,3], cov[2,3] = parse(Float64, msm[5]), parse(Float64, msm[6]), parse(Float64, msm[8])
    cov += cov'
    cov[1,1], cov[2,2], cov[3,3] = parse(Float64, msm[4]), parse(Float64, msm[7]), parse(Float64, msm[9])
    zi = zeros(3,1)
    zi[:,1] = [parse(msm[1]);parse(msm[2]);parse(msm[3])]
    return PriorPose2(zi, cov^2, [1.0])
  elseif lkl[1]=="PTPR2"
    msm = split(elem["meas"], ' ')
    cov = zeros(2,2)
    cov[1,2] = parse(Float64, msm[4])^2
    cov += cov'
    cov[1,1], cov[2,2] = parse(Float64, msm[3])^2, parse(Float64, msm[5])^2
    zi = zeros(2)
    zi[1:2] = [parse(msm[1]);parse(msm[2])]
    return PriorPoint2D(zi, cov, [1.0])
  elseif lkl[1]=="PP2"
    msm = split(elem["meas"], ' ')
    cov = zeros(3,3)
    cov[1,2], cov[1,3], cov[2,3] = parse(Float64, msm[5]), parse(Float64, msm[6]), parse(Float64, msm[8])
    cov += cov'
    cov[1,1], cov[2,2], cov[3,3] = parse(Float64, msm[4]), parse(Float64, msm[7]), parse(Float64, msm[9])
    zij = zeros(3,1)
    zij[:,1] = [parse(msm[1]);parse(msm[2]);parse(msm[3])]
    return Pose2Pose2(zij, cov^2, [1.0])
  elseif lkl[1]=="BR"
    msm = split(elem["meas"], ' ')
    return Pose2DPoint2DBearingRange{Normal, Normal}(
                  Normal(parse(msm[1]), parse(Float64, msm[3]) ),
                  Normal(parse(msm[2]),parse(Float64, msm[5]) )  )
  elseif lkl[1]=="rangeBearingMEAS"
    # @show elem["range"]
    rngs = Vector{Float64}(elem["range"] )
    bearing = Vector{Float64}(elem["bearing"] )
    @show size(rngs), size(bearing)
    prange = resample(kde!(rngs),N)
    pbear = resample(kde!(bearing),N)
    # warn("temporary return")
    # return prange, pbear
    return Pose2DPoint2DBearingRangeDensity(pbear, prange)
  elseif lkl[1]=="rangeMEAS"
    # @show elem["range"]
    rngs = Vector{Float64}(elem["range"] )
    @show size(rngs)
    prange = resample(kde!(rngs),N)
    # warn("temporary return")
    # return prange
    return Pose2DPoint2DRangeDensity(prange)
  else
    return error("Don't know how to convert $(lkl[1]) to a factor")
  end
end


function populatenewvariablenodes!(fgl::FactorGraph, newvertdict::SortedDict; N::Int=100)

  for (neoNodeId,elem) in newvertdict
    # @show neoNodeId
    nlbsym = Symbol()
    uidl = 0
    labels = AbstractString["$(fgl.sessionname)"]
    initvals = Array{Float64,2}()
    if elem[:frtend]["t"] == "P"
      uidl = elem[:frtend]["uid"]+1 # TODO -- remove +1 and allow :x0, :l0
      nlbsym = Symbol(string('x', uidl))
      initvals = 0.1*randn(3,N) # TODO -- fix init to proper values
      # v = addNode!(fgl, nlbsym, , 0.01*eye(3), N=N, ready=0, uid=uidl,api=localapi)
      push!(labels,"POSE")
    elseif elem[:frtend]["t"] == "L"
      warn("using hack counter for LANDMARKS uid +200000")
      uidl = elem[:frtend]["tag_id"]+200000 # TODO complete hack
      nlbsym = Symbol(string('l', uidl))
      initvals = 0.1*randn(2,N)
      push!(labels,"LANDMARK")
    end
    if !haskey(fgl.IDs, nlbsym) && nlbsym != Symbol()
      @show nlbsym, size(initvals)
      v = addNode!(fgl, nlbsym, initvals, 0.01*eye(size(initvals,2)), N=N, ready=0, uid=uidl,api=localapi) # TODO remove initstdev, deprecated
      insertValuesCloudVert!(fgl, neoNodeId, elem, uidl, v, labels=labels)
    end
  end
  nothing
end

function populatenewfactornodes!(fgl::FactorGraph, newvertdict::SortedDict)
  warn("using hack counter for FACTOR uid +100000")
  fuid = 100000 # offset factor values
  # neoNodeId = 63363
  # elem = newvertdict[neoNodeId]
  for (neoNodeId,elem) in newvertdict
    if elem[:frtend]["t"] == "F"
      # verts relating to this factor
      @show neoNodeId
      verts = Vector{Graphs.ExVertex}()
      i=0
      for bf in split(elem[:frtend]["btwn"], ' ')
        i+=1
        # uid = 0
        uid = parse(Int,bf)+1
        if ((elem[:frtend]["lklh"][1:2] == "BR" ||
            elem[:frtend]["lklh"] == "rangeBearingMEAS" ||
            elem[:frtend]["lklh"] == "rangeMEAS" ) && i==2 ) ||
            ( elem[:frtend]["lklh"] == "PTPR2 G 2 STDEV" && i==1 )
          # detect bearing range factors being added between pose and landmark
          warn("using hack counter for LANDMARKS uid +200000")
          uid = parse(Int,bf)+200000 # complete hack
        end
        push!(verts, fgl.g.vertices[uid])
      end
      # the factor type
      usrfnc = recoverConstraintType(elem[:frtend])
      fuid += 1
      vert = addFactor!(fgl, verts, usrfnc, ready=0, api=localapi, uid=fuid, autoinit=true)
      println("at populatenewfactornodes!, btwn="*elem[:frtend]["btwn"])
      insertValuesCloudVert!(fgl, neoNodeId, elem, fuid, vert, labels=["FACTOR";"$(fgl.sessionname)"])

      # TODO -- upgrade to add multple variables
      # for vv in verts
      vv = verts[end]
      @show vv.label, Base.mean(vv.attributes["data"].val, 2)
      dlapi.updatevertex!(fgl, vv, updateMAPest=false)
      # end
    end
  end
  nothing
end


"""
    updatenewverts!(fgl; N=100)

Convert vertices of session in Neo4j DB with Caesar.jl's required data elements
in preparation for MM-iSAMCloudSolve process.
"""
function updatenewverts!(fgl::FactorGraph; N::Int=100)
  sortedvd = getnewvertdict(fgl.cg.neo4j.connection, fgl.sessionname)
  populatenewvariablenodes!(fgl, sortedvd, N=N)
  populatenewfactornodes!(fgl, sortedvd)
  nothing
end


"""
    resetentireremotesession(conn, session)

match (n:\$(session))
remove n.backendset, n.ready, n.data, n.bigData, n.label, n.packedType, n.exVertexId, n.shape, n.width
set n :NEWDATA
return n
"""
function resetentireremotesession(conn, session)
  loadtx = transaction(conn)
  query = "match (n:$(session))
           remove n.backendset, n.ready, n.data, n.bigData, n.label, n.packedType, n.exVertexId, n.shape, n.width, n.MAP_est
           set n :NEWDATA"
  cph = loadtx(query, submit=true)
  loadresult = commit(loadtx)
  nothing
end


  #
