# Convert slamindb Functions




function getmaxfactorid(conn, session::AbstractString)
  loadtx = transaction(conn)
  query =  "match (n:$(session):FACTOR)
            where not (n:NEWDATA)
            with id(n) as idn, n.exVertexId as nexvid
            order by nexvid desc limit 1
            return idn, nexvid"
  cph = loadtx(query, submit=true)
  exvid = 0
  if length(cph.results[1]["data"]) > 0
    neoid = cph.results[1]["data"][1]["row"][1]
    exvid = cph.results[1]["data"][1]["row"][2]
  end
  return exvid
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

  newvertdict = DataStructures.SortedDict{Int, Dict{Symbol, Dict{AbstractString,Any}}, Base.Order.ForwardOrdering}(Base.Order.ForwardOrdering())
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

"""
    parseMergeVertAttr(v, elem)

Parse elem dictionary according to thin ```frtend``` interface from other languages,
and merge contents into attributes of ```v::Graphs.ExVertex```.
"""
function parseMergeVertAttr!(v::Graphs.ExVertex, elem)
  mongos = Dict()
  for (k,va) in elem
    if k == :frtend
      v.attributes[string(k)] = JSON.json(va)
    elseif k == :mongo_keys
      mongos = JSON.parse(va["val"])
      v.attributes[string(k)] = va["val"]
    elseif k == :ready
      v.attributes[string(k)] = typeof(va["val"]) == Int ? va["val"] : parse(Int,va["val"])
    else
      warn("setting $(k) to $(typeof(va["val"]))")
      v.attributes[string(k)] = va["val"]  # this is replacing data incorrectly
    end
  end
  return mongos
end

"""
    mergeCloudVertex!(...)

Hodgepodge function to merge data in CloudVertex
"""
function mergeCloudVertex!{T <: AbstractString}(cg::CloudGraph,
      v::Graphs.ExVertex,
      neoNodeId::Int,
      alreadyexists::Bool,
      neo4jNode,
      existlbs::Vector{AbstractString},
      mongos;
      labels::Vector{T}=String[]  )
  #
  cv = CloudVertex()
  if alreadyexists
    # simply fetch existing cloudgraph if it exists, AGAIN
    cv = CloudGraphs.get_vertex(cg, neoNodeId)
    cgv = cloudVertex2ExVertex(cv)
    # NOTE, overwrite all values, assuming this is ONLY HAPPENING WITH VARIABLENODES
    # not checking to unpack Packed types
    for (ke,val) in cgv.attributes
      v.attributes[ke] = val
    end
  else
    cv = exVertex2CloudVertex( v )
    cv.neo4jNode = neo4jNode
    cv.neo4jNodeId = neoNodeId
    # can't fetch in cloudgraphs with thin interface: (fntend, mongo_keys, ready)
    cv.isValidNeoNodeId = true
    filter!(e->e!="NEWDATA",existlbs)
    cv.labels = union(existlbs, labels)
  end

  # want to check the mongo keys anyway, since they could have changed
  mergeBigDataElements!(cv.bigData.dataElements, mongos)
  return cv
end


function mergeBigDataElements!(bdes::Vector{BigDataElement}, mongos::Dict)
  if mongos != nothing
    for (k,oid) in mongos

      haskey = false
      for bd in bdes
        if bd.description == k
          warn("skipping bigDataElement $(bd.description), assumed to be repeat during merge.")
          haskey = true
        end
      end
      if !haskey
        println("transfer mongo oid $((k,oid))")
        # empty data, since we will call update_NeoBigData and not save_BigData
        bdei = CloudGraphs.BigDataElement(k, Vector{UInt8}(), string(oid))
        push!(bdes, bdei);
      end
    end
  # else
  #   println("no mongo")
  end
  nothing
end

function mergeValuesIntoCloudVert!{T <: AbstractString}(fgl::FactorGraph,
      neoNodeId::Int,
      elem,
      uidl,
      v::Graphs.ExVertex;
      labels::Vector{T}=String[]  )
  #

  # why am I getting a node again (because we don't have the labels here)?
  # TODO -- pass labels via elem dictionary
  neo4jNode = Neo4j.getnode(fgl.cg.neo4j.graph, neoNodeId)
  existlbs = Vector{AbstractString}(neo4jNode.metadata["labels"])

  alreadyexists = sum(existlbs .== "NEWDATA") == 0

  # parse dictionary of values retrieved from Neo4j
  mongos = parseMergeVertAttr!(v, elem)

  if !haskey(fgl.cgIDs, uidl)
    fgl.cgIDs[uidl] = neoNodeId
  else
    fgl.cgIDs[uidl] == neoNodeId ? nothing : error("trying to merge neo ids $(fgl.cgIDs[uidl]) and $(neoNodeId)")
  end

  # merge CloudVertex
  cv = mergeCloudVertex!(fgl.cg, v, neoNodeId, alreadyexists, neo4jNode, existlbs, mongos, labels=labels )

  # update merged vertex to database
  CloudGraphs.update_vertex!(fgl.cg, cv, true)
  # CloudGraphs.update_NeoBigData!(fgl.cg, cv)
  nothing
end

function parsePose2Pose2Constraint(lkl, elem)
  msm = split(elem["meas"], ' ')
  cov = zeros(3,3)
  cov[1,2], cov[1,3], cov[2,3] = parse(Float64, msm[5]), parse(Float64, msm[6]), parse(Float64, msm[8])
  cov += cov'
  cov[1,1], cov[2,2], cov[3,3] = parse(Float64, msm[4]), parse(Float64, msm[7]), parse(Float64, msm[9])
  zij = zeros(3,1)
  zij[:,1] = [parse(msm[1]);parse(msm[2]);parse(msm[3])]
  if length(lkl) >=4
    if lkl[4] == "STDEV"
      cov = cov^2
    end
  end
  return Pose2Pose2(zij, cov, [1.0])
end

function recoverConstraintType(cgl::CloudGraph,
            elem;
            mongokeys::Dict=Dict(),
            N::Int=200  )
  #
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
    return parsePose2Pose2Constraint(lkl, elem)
    # msm = split(elem["meas"], ' ')
    # cov = zeros(3,3)
    # cov[1,2], cov[1,3], cov[2,3] = parse(Float64, msm[5]), parse(Float64, msm[6]), parse(Float64, msm[8])
    # cov += cov'
    # cov[1,1], cov[2,2], cov[3,3] = parse(Float64, msm[4]), parse(Float64, msm[7]), parse(Float64, msm[9])
    # zij = zeros(3,1)
    # zij[:,1] = [parse(msm[1]);parse(msm[2]);parse(msm[3])]
    # return Pose2Pose2(zij, cov^2, [1.0])
  elseif lkl[1]=="BR"
    msm = split(elem["meas"], ' ')
    return Pose2DPoint2DBearingRange{Normal, Normal}(
                  Normal(parse(msm[1]), parse(Float64, msm[3]) ),
                  Normal(parse(msm[2]),parse(Float64, msm[5]) )  )
  elseif lkl[1]=="rangeBearingMEAS"
    @show rangekey = mongokeys["range"]
    rngs = bin2arr(CloudGraphs.read_MongoData(cgl, rangekey))
    # rngs = Vector{Float64}(elem["range"] )
    @show bearingkey = mongokeys["bearing"]
    bearing = bin2arr(CloudGraphs.read_MongoData(cgl, bearingkey))
    # bearing = Vector{Float64}(elem["bearing"] )
    @show size(rngs), size(bearing)
    prange = resample(kde!(rngs),N)
    pbear = resample(kde!(bearing),N)
    # warn("temporary return")
    # return prange, pbear
    return Pose2DPoint2DBearingRangeDensity(pbear, prange)
  elseif lkl[1]=="rangeMEAS"
    @show rangekey = mongokeys["range"]
    rngs = bin2arr(CloudGraphs.read_MongoData(cgl, rangekey))
    # rngs = Vector{Float64}(elem["range"] )
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
      initvals = 0.1*randn(2,N) # Make sure autoinit still works properly
      push!(labels,"LANDMARK")
    end
    if !haskey(fgl.IDs, nlbsym) && nlbsym != Symbol()
      # @show nlbsym, size(initvals)
      # TODO remove initstdev, deprecated
      v = addNode!(fgl, nlbsym, initvals, 0.01*eye(size(initvals,2)), N=N, ready=0, uid=uidl,api=localapi)
      mergeValuesIntoCloudVert!(fgl, neoNodeId, elem, uidl, v, labels=labels)
      println()
    end
  end
  println("done populating new variables")
  nothing
end

function populatenewfactornodes!(fgl::FactorGraph, newvertdict::SortedDict, maxfuid::Int)
  foffset = 100000
  fuid = maxfuid >= foffset ? maxfuid : maxfuid+foffset # offset factor values
  warn("using hack counter for FACTOR uid starting at $(fuid)")
  # neoNodeId = 63363
  # elem = newvertdict[neoNodeId]
  for (neoNodeId,elem) in newvertdict
    if elem[:frtend]["t"] == "F"
      # @show neoNodeId
      if !haskey(elem,:ready)
        # warn("missing ready field")
        continue
      end
      if Int(elem[:ready]["val"]) != 1
        warn("ready/val field not equal to 1")
        continue
      end
      # verts relating to this factor
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
          warn("using hack counter for LANDMARKS uid +$(2*foffset)")
          uid = parse(Int,bf)+ 2*foffset # complete hack
        end
        push!(verts, fgl.g.vertices[uid])
      end
      # the factor type
      usrfnc = !haskey(elem,:mongo_keys) ? recoverConstraintType(fgl.cg, elem[:frtend]) : recoverConstraintType(fgl.cg, elem[:frtend], mongokeys=JSON.parse(elem[:mongo_keys]["val"]) )
      fuid += 1
      vert = addFactor!(fgl, verts, usrfnc, ready=0, api=localapi, uid=fuid, autoinit=true)
      println("at populatenewfactornodes!, btwn="*elem[:frtend]["btwn"])
      mergeValuesIntoCloudVert!(fgl, neoNodeId, elem, fuid, vert, labels=["FACTOR";"$(fgl.sessionname)"])

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
    updatenewverts!(fgl::FactorGraph; N::Int)

Convert vertices of session in Neo4j DB with Caesar.jl's required data elements
in preparation for MM-iSAMCloudSolve process.
"""
function updatenewverts!(fgl::FactorGraph; N::Int=100)
  sortedvd = getnewvertdict(fgl.cg.neo4j.connection, fgl.sessionname)
  populatenewvariablenodes!(fgl, sortedvd, N=N)
  maxfuid = getmaxfactorid(fgl.cg.neo4j.connection, fgl.sessionname)
  populatenewfactornodes!(fgl, sortedvd, maxfuid)
  nothing
end


"""
    resetentireremotesession(conn, session)

match (n:\$(session))
remove n.backendset, n.ready, n.data, n.bigData, n.label, n.packedType, n.exVertexId, n.shape, n.width
set n :NEWDATA
return n
"""
function resetentireremotesession(conn, session::AbstractString; segment::AbstractString="")
  loadtx = transaction(conn)
  query = segment == "" ? "match (n:$(session)) " : "match (n:$(session):$(segment)) "
  query = query*"where exists(n.frtend)
           remove n.backendset, n.ready, n.data, n.bigData, n.label, n.packedType, n.exVertexId, n.shape, n.width, n.MAP_est
           set n :NEWDATA"
  cph = loadtx(query, submit=true)
  loadresult = commit(loadtx)
  nothing
end





#
