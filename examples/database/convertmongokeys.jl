
# function to convert elements in mongo_keys into bigData CloudGraphs container

using Caesar, CloudGraphs, Neo4j
using LibBSON, Mongo
using JSON

using IncrementalInference


using PyCall

# lcmtpath = joinpath(dirname(@__FILE__),"python")
# unshift!(PyVector(pyimport("sys")["path"]),lcmtpath)
# include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
# collection = "bindata"

@pyimport numpy as np
@pyimport cv2 as opencv

include("VisualizationUtilities.jl") # @pyimport getimages as gi


# # connect to the server, CloudGraph stuff
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
session = "SESSTURT45"


configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, dbusr, dbpwd, mongoaddress, 27017, false, "", "");
cloudGraph = connect(configuration);
conn = cloudGraph.neo4j.connection
registerGeneralVariableTypes!(cloudGraph)
Caesar.usecloudgraphsdatalayer!()


# also connect to mongo separately
client = pymongo.MongoClient(mongoaddress)
db = client[:CloudGraphs]


N=100
fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)


# totally reset to frontend built state in DB
# resetentireremotesession(conn,session)


# updatenewverts!(fg, N=N)
# --
# sortedvd = getnewvertdict(fg.cg.neo4j.connection, fg.sessionname)
# populatenewvariablenodes!(fg, sortedvd, N=N)


fullLocalGraphCopy!(fg, conn)

IDs = getPoseExVertexNeoIDs(conn, sessionname=session, reqbackendset=false);


xx,ll = ls(fg)
LD = Array{Array{Float64,1},1}()
C = Vector{AbstractString}()

lbl = xx[2]


# function getmongokeys(fgl::FactorGraph, x::Symbol, IDs)
#   cvid = -1
#   for id in IDs
#     if Symbol(fgl.g.vertices[id[1]].label) == x
#       cvid = id[2]
#       break
#     end
#   end
#   # @show cvid
#   cv = CloudGraphs.get_vertex(fgl.cg, cvid)
#   jsonstr = cv.properties["mongo_keys"]
#   return JSON.parse(jsonstr)
# end

mongk = getmongokeys(fg, lbl, IDs)

img = fetchmongoimg(db[collection], mongk["depthframe_image"])
# mongo_keydepth = bson.ObjectId(mongk["depthframe_image"])
# dpim, ims = gi.fastdepthimg(db[collection], mongo_keydepth)






# ===============================================================

# get the mongokeys from Neo






# ===============================================================
using PyCall

lcmtpath = joinpath(dirname(@__FILE__),"python")
unshift!(PyVector(pyimport("sys")["path"]),lcmtpath)
include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))

@show mongoaddress

@pyimport bson
@pyimport pymongo

@pyimport numpy as np
@pyimport cv2 as opencv

@pyimport getimages as gi


collection = "bindata"
key = "58af67265d7625647859fa73"
@show gi.test(key,1)

# @show gi.getrgbimg(mongoaddress, collection, key)


client = pymongo.MongoClient(mongoaddress)
db = client[:CloudGraphs]

mongo_key = bson.ObjectId(key)

im = gi.fastrgbimg(db[collection], mongo_key)
@show size(im),typeof(im)
# opencv.imshow("image", im)


keydepth = "58b07be05d7625699151a01d"
mongo_keydepth = bson.ObjectId(keydepth)

dpim, ims = gi.fastdepthimg(db[collection], mongo_keydepth)





results = (db[collection])[:find]( Dict{String, Any}("_id" => mongo_key) )

for rec in results
  @show rec["description"]
  # im = rec["val"]
  val = "np.fromstring(str($(rec)))"
  pycall()
end



# =============================================================



mongoId = BSONOID("58af67265d7625647859fa73")

im = []

val = find(fg.cg.mongo.cgBindataCollection, query("_id" => mongoId))
elem = first(val)

val = elem["val"]

@show typeof(val)

# unsafe_load(val,1) # doesn't work

LibBSON.libbson








for doc in find(fg.cg.mongo.cgBindataCollection, query("_id" => mongoId))
  im = doc["val"]
end

numNodes = count(fg.cg.mongo.cgBindataCollection, ("_id" => mongoId));
if numNodes != 1
  error("The query for $(mongoId) returned $(numNodes) values, expected a 1-to-1 mapping!");
end

results = first(find(fg.cg.mongo.cgBindataCollection, ("_id" => eq(mongoId))) )

data = CloudGraphs._stringToUint8Array(results["val"]);

@show length(results)


o = next(results, nothing)


for (k,v) in results
  @show k
end

dr = dict(results)








id = slam.fg.IDs[sp2[1]]
cgid = slam.fg.cgIDs[id]
cv = CloudGraphs.get_vertex(slam.fg.cg, cgid)
# for f in sp2[2:end]
fid = open(sp2[2],"r")
imageData = readbytes(fid) # imageData::Vector{UInt8}
close(fid)
bdei = CloudGraphs.BigDataElement("keyframe-image", imageData)
push!(cv.bigData.dataElements, bdei);

fid = open(sp2[3],"r")
imageData = readbytes(fid) # imageData::Vector{UInt8}
close(fid)
bdei = CloudGraphs.BigDataElement("depthframe-image", imageData)
push!(cv.bigData.dataElements, bdei);

CloudGraphs.save_BigData!(slam.fg.cg, cv)




cursor = find(cloudGraph.mongo.cgBindataCollection, query())
for o in cursor
    # println( ", oid: ", o["_id"])
    println("id: ", o["_id"]);
end




#
