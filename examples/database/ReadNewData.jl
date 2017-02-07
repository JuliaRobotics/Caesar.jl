# find and interact with NEWDATA in neo4j


using Caesar, RoME, IncrementalInference
using CloudGraphs, Neo4j
using Distributions
using JSON

# # # connect to the server, CloudGraph stuff
# dbaddress = length(ARGS) > 0 ? ARGS[1] : "localhost"
# println("Taking Neo4j database address as $(dbaddress)...")
#
# dbusr = length(ARGS) > 1 ? ARGS[2] : ""
# dbpwd = length(ARGS) > 2 ? ARGS[3] : ""
#
# mongoaddress = length(ARGS) > 3 ? ARGS[4] : "localhost"
# println("Taking Mongo database address as $(mongoaddress)...")
#
# session = length(ARGS) > 4 ? string(ARGS[5]) : ""
# println("Attempting to draw session $(session)...")
#
# DRAWDEPTH = length(ARGS) > 5 ? ARGS[6]=="drawdepth" : false

# TODO comment out for command line operation
include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
session = "SESSROX"


# Connection to database
configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, dbusr, dbpwd, mongoaddress, 27017, false, "", "");
cloudGraph = connect(configuration);
conn = cloudGraph.neo4j.connection
# register types of interest in CloudGraphs
registerGeneralVariableTypes!(cloudGraph)
Caesar.usecloudgraphsdatalayer!()


loadtx = transaction(conn)
query = "match (n:$(session))-[r1]-(f:NEWDATA:$(session):FACTOR)-[r2]-(m:NEWDATA:$(session)) return distinct n, f, m"
cph = loadtx(query, submit=true)
# loadresult = commit(loadtx)

# @show cph.results[1]

newvertdict = Dict{Int, Dict{AbstractString,Any}}()

for val in cph.results[1]["data"]
  i = 0
  for elem in val["meta"]
    # @show elem["type"]    # @show rdict["type"]
    i+=1
    rdict = JSON.parse(val["row"][i]["frtend"])
    newvertdict[elem["id"]] = rdict
    # if uppercase(rdict["type"])=="POSE" || uppercase(rdict["type"])=="FACTOR"
      # npsym = Symbol(string("x",parse(Int, rdict["userid"])+1)) # TODO -- fix :x0 requirement
  end
  # println()
end

# @show newvertdict

for elem in newvertdict
  @show elem[2]["t"]
  @show collect(keys(elem[2]))
  if elem[2]["t"] == "P"
    @show elem[1], elem[2]["uid"]
  elseif elem[2]["t"] == "L"
    @show elem[1], elem[2]["uid"], elem[2]["tag_id"]
  end
  println()
end


function insertValuesCloudVert(fgl::FactorGraph, neoNodeId::Int, elem, uidl, nlbsym; labels=String[])
  #
  addNode!(fgl, nlbsym, 0.1*randn(3,N), 0.01*eye(3), N=N, ready=0, uid=uidl,api=localapi)

  v = getVert(fgl, nlbsym, api=localapi)
  warn("deleting frntend values")
  delete!(v.attributes,"frntend")
  # v.attributes["frntend"] = elem
  fgl.cgIDs[uidl] = neoNodeId

  # updateFullCloudVertData!(fgl, v)
  #
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


function recoverConstraintType(elem)
  lkl = split(elem["lklh"], ' ')
  if lkl[1]=="PR2"
    msm = split(elem["meas"], ' ')
    cov = zeros(3,3)
    cov[1,2], cov[1,3], cov[2,3] = parse(Float64, msm[5]), parse(Float64, msm[6]), parse(Float64, msm[8])
    cov += cov'
    cov[1,1], cov[2,2], cov[3,3] = parse(Float64, msm[4]), parse(Float64, msm[7]), parse(Float64, msm[9])
    zi = zeros(3,1)
    zi[:,1] = [parse(msm[1]);parse(msm[2]);parse(msm[3])]
    return PriorPose2(zi, cov, [1.0])
  elseif lkl[1]=="PP2"
    msm = split(elem["meas"], ' ')
    cov = zeros(3,3)
    cov[1,2], cov[1,3], cov[2,3] = parse(Float64, msm[5]), parse(Float64, msm[6]), parse(Float64, msm[8])
    cov += cov'
    cov[1,1], cov[2,2], cov[3,3] = parse(Float64, msm[4]), parse(Float64, msm[7]), parse(Float64, msm[9])
    zij = zeros(3,1)
    zij[:,1] = [parse(msm[1]);parse(msm[2]);parse(msm[3])]
    return Pose2Pose2(zij, cov, [1.0])
  elseif lkl[1]=="BR"
    msm = split(elem["meas"], ' ')
    return Pose2DPoint2DBearingRange{Normal, Normal}(
                  Normal(parse(msm[1]), parse(Float64, msm[3]) ),
                  Normal(parse(msm[2]),parse(Float64, msm[5]) )  )
  else
    return error("Don't know how to convert $(lkl[1]) to a factor")
  end
end

# next step is to convert theis data into actual usable graph for IIF.
# We'll do this with CloudGraphs.updatevertex!(...)

# first iteration, lets use addNode/addFactor to local graph only and
# then call updateFullCloudVertData!. ID tracking may be the issue here.

N=100

fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)

for (neoNodeId,elem) in newvertdict
  # neoNodeId = 31369
  if elem["t"] == "P"
    uidl = elem["uid"]+1
    nlbsym = Symbol(string('x', uidl))
    insertValuesCloudVert(fg, neoNodeId, elem, uidl, nlbsym, labels=["POSE";"$(session)"])
  elseif elem["t"] == "L"
    uidl = elem["uid"]+1
    nlbsym = Symbol(string('l', uidl))
    insertValuesCloudVert(fg, neoNodeId, elem, uidl, nlbsym, labels=["LANDMARK";"$(session)"])
  end
end



warn("using hack counter for FACTOR uid")
fuid = 10000
for (neoNodeId,elem) in newvertdict
  if elem["t"] == "F"
    # verts relating to this factor
    verts = Vector{Graphs.ExVertex}()
    for bf in split(elem["btwn"], ' ')
      uid = parse(Int,bf)+1
      push!(verts, fg.g.vertices[uid])
    end
    # the factor type
    usrfnc = recoverConstraintType(elem)
    fuid += 1
    vert = addFactor!(fg, verts, usrfnc, ready=0, api=localapi, uid=fuid)

    fg.cgIDs[fuid] = neoNodeId

    # updateFullCloudVertData!(fg, v)
    #
    cv = exVertex2CloudVertex( vert )
    cv.neo4jNodeId = neoNodeId
    cv.neo4jNode = Neo4j.getnode(fg.cg.neo4j.graph, neoNodeId)
    cv.isValidNeoNodeId = true
    # cv.labels=labels
    #
    CloudGraphs.update_vertex!(fg.cg, cv)

  end
end


# function addFactor!{I <: Union{FunctorInferenceType, InferenceType}, T <: AbstractString}(fgl::FactorGraph,
#       Xi::Array{Graphs.ExVertex,1},
#       usrfnc::I;
#       ready::Int=1,
#       api::DataLayerAPI=dlapi,
#       labels::Vector{T}=String[] )


ls(fg)















#
