# help DBCollectionsViewerService
using PyCall

@pyimport bson
@pyimport pymongo


lcmtpath = joinpath(dirname(@__FILE__),"python")
unshift!(PyVector(pyimport("sys")["path"]),lcmtpath)
include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
collection = "bindata"


# temporary helper function to read binary BSON data via pymongo
@pyimport getimages as gi



# also connect to mongo separately
client = pymongo.MongoClient(mongoaddress)
db = client[:CloudGraphs]



function getmongokeys(fgl::FactorGraph, x::Symbol, IDs)
  cvid = -1
  for id in IDs
    if Symbol(fgl.g.vertices[id[1]].label) == x
      cvid = id[2]
      break
    end
  end
  # @show cvid
  cv = CloudGraphs.get_vertex(fgl.cg, cvid)
  if haskey(cv.properties, "mongo_keys")
    jsonstr = cv.properties["mongo_keys"]
    return JSON.parse(jsonstr), cvid
  else
    return Dict{AbstractString, Any}(), cvid
  end
end


function fetchmongodepthimg(dbcoll, key)
  mongo_keydepth = bson.ObjectId(key)
  dpim, ims = gi.fastdepthimg(dbcoll, mongo_keydepth)
  return dpim
end

function fetchmongorgbimg(dbcoll, key)
  mongo_keydepth = bson.ObjectId(key)
  dpim, ims = gi.fastdepthimg(dbcoll, mongo_keydepth)
  return dpim
end








#
