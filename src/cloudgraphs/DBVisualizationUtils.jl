# help DBCollectionsViewerService
# using PyCall

thispath = joinpath(joinpath(dirname(@__FILE__),".."),"python")
println("Adding to session Python path: $(thispath)")
unshift!(PyVector(pyimport("sys")["path"]),thispath)
# include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
# collection = "bindata"


# temporary helper function to read binary BSON data via pymongo
@pyimport bson
@pyimport pymongo
@pyimport getimages as gi



function getmongokeys(fgl::FactorGraph, x::Symbol, IDs)
  cvid = -1
  # TODO -- could likely just use existing mapping already in fgl
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


function getbinarraymongo(dbcoll, key)
  mongo_key = bson.ObjectId(key)
  arr = gi.getbinarray(dbcoll, mongo_key)
  return arr
end






#
