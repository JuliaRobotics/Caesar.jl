# help DBCollectionsViewerService
# using PyCall

# thispath = joinpath(joinpath(dirname(@__FILE__),".."),"python")
# println("Adding to session Python path: $(thispath)")
# unshift!(PyVector(pyimport("sys")["path"]),thispath)
# # include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
# # collection = "bindata"
#
#
# # temporary helper function to read binary BSON data via pymongo
# @pyimport bson
# @pyimport pymongo
# @pyimport getimages as gi



function getmongokeys(fgl::FactorGraph, x::Symbol, IDs)
  cvid = -1
  # TODO -- could likely just use existing mapping already in fgl
  for id in IDs
    if Symbol(fgl.g.vertices[id[1]].label) == x
      cvid = id[2]
      break
    end
  end
  if cvid == -1
    warn("getmongokeys is not finding $(x)")
    return Dict{AbstractString, Any}(), cvid
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

function fetchmongorgbimg(cg::CloudGraph, key::AbstractString)
  myKeyToFind = BSONOID(key)
  findsomthing = find(cg.mongo.cgBindataCollection, ("_id" => eq(myKeyToFind)))
  myFavouriteKey = first( findsomthing );
  data = myFavouriteKey["val"]
  img = ImageMagick.readblob(data);

  # r, c = size(img)
  # imgA = zeros(r,c,3);
  # for i in 1:r, j in 1:c
  #   imgA[i,j,1] = img[i,j].r
  #   imgA[i,j,2] = img[i,j].g
  #   imgA[i,j,3] = img[i,j].b
  # end
  return img
end

function fetchmongodepthimg(cg::CloudGraph, key::AbstractString; dtype::DataType=Float64)
  myKeyToFind = BSONOID(key) # some valid numbers

  findsomthing = find(cg.mongo.cgBindataCollection, ("_id" => eq(myKeyToFind)))
  myFavouriteKey = first( findsomthing );
  mfkv = myFavouriteKey["val"];

  len = length(mfkv)
  dl = sizeof(dtype)
  alen = round(Int,len/dl)

  ptrT = pointer(mfkv)  # Ptr{UInt8}
  ptrTf = convert(Ptr{dtype}, ptrT)  # Ptr{Float32}
  arr = Vector{dtype}(alen);

  unsafe_copy!(pointer(arr),ptrTf,alen)

  return arr
end

# function fetchmongodepthimg(dbcoll, key)
#   mongo_keydepth = bson.ObjectId(key)
#   dpim, ims = gi.fastdepthimg(dbcoll, mongo_keydepth)
#   return dpim
# end
#
#
# function fetchmongorgbimg(dbcoll, key)
#   mongo_keydepth = bson.ObjectId(key)
#   dpim, ims = gi.fastdepthimg(dbcoll, mongo_keydepth)
#   return dpim
# end
#
#
# function getbinarraymongo(dbcoll, key)
#   mongo_key = bson.ObjectId(key)
#   arr = gi.getbinarray(dbcoll, mongo_key)
#   return arr
# end






#
