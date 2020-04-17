using Mongo, LibBSON
using Caesar

include(joinpath(dirname(@__FILE__),"..","database","blandauthremote.jl"))
cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)
# cloudGraph, addrdict = standardcloudgraphsetup()

## interactive operation
# include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))

# Get the keys (and possibly the data)
# cursor = find(cloudGraph.mongo.cgBindataCollection, query())
# for o in cursor
#     # println( ", oid: ", o["_id"])
#     println("id: ", o["_id"]);
# end

# Get Roxana's encoded parameters
#REF: https://github.com/dehann/Caesar.jl/blob/master/examples/database/python/mongo_interaction.py
# mongoClient = cloudGraph.mongo.client
# mongoTestCollection = MongoCollection(mongoClient, "tester", "testing_collection")
# cursor = find(mongoTestCollection, query())
# for o in cursor
#     println("Key: ", o["key"], ", oid: ", o["_id"])  # Or simply,
#     #println(o)
# end

# example of string array
# myKeyToFind = BSONOID("58b0a34b5d76256bd2b8183c") # all NaNs
myKeyToFind = BSONOID("58b0a3525d76256bd2b81860") # some valid numbers

findsomthing = find(cloudGraph.mongo.cgBindataCollection, ("_id" => eq(myKeyToFind)))
myFavouriteKey = first( findsomthing );
mfkv = myFavouriteKey["val"];
# @show myFavouriteKey["val"]

ptrT = pointer(mfkv)
# Ptr{UInt8} @0x00000000080f9740
#
ptrTf = convert(Ptr{Float32}, ptrT)
# Ptr{Float32} @0x00000000080f9740

len = length(mfkv)
arr = Vector{Float32}(round(Int,len/4));

unsafe_copy!(pointer(arr),ptrTf,round(Int,len/4))



# fetch a binary png image
myKeyToFind = BSONOID("58af67255d7625647859fa71") # good example of opencv binary encoded png
findsomthing = find(cloudGraph.mongo.cgBindataCollection, ("_id" => eq(myKeyToFind)))
# @show myFavouriteKey["val"]
myFavouriteKey = first( findsomthing );
gf
file = open("test.png", "w")
write(file, myFavouriteKey["val"])
close(file)

run(`eog test.png`)

data = myFavouriteKey["val"]
using ImageMagick
img = ImageMagick.readblob(data);
# using Images
# using FileIO
# using Images, ImageView
# img = load("test.png")
# imshow(img)
# run(`eog test.png`)
# run(`rm test.png`)

img[1,1].r
imgA = zeros(480,640,3);
@time for i in 1:480, j in 1:640
  imgA[i,j,1] = img[i,j].r
  imgA[i,j,2] = img[i,j].g
  imgA[i,j,3] = img[i,j].b
end




# limitations in LibBSON.jl prevent Binary unpacking at this time, using PyCall to pymongo interim
# key = BSONOID("58c1a1bdb2ca1534d30b1c96")
# obj = find(cloudGraph.mongo.cgBindataCollection, ( "_id" => eq(key) ) )

# fetching data from Mongo using getimages.py
# IDs = getPoseExVertexNeoIDs(fg.cg.neo4j.connection, session=attrdict["session"], reqbackendset=false);
# mongk = getmongokeys(fg, :x1, IDs)
# mongo_key = bson.ObjectId(mongk["keyframe_rgb"])
# arr = gi.getbinarray(dbcoll, mongo_key)


myKeyToFind = BSONOID("58e302546689bc617c1994c1") # good example of opencv binary encoded png
findsomthing = find(cloudGraph.mongo.cgBindataCollection, ("_id" => eq(myKeyToFind)))
# @show myFavouriteKey["val"]
mkf = first( findsomthing );

typeof(mkf["val"])

buf = IOBuffer(mkf["val"])

st = String(take!(buf))
bb = BSONObject(st)
typeof(bb)
bba = convert(Array, bb["pointcloud"][1])
convert(bb["pointcloud"]



# temporary helper function to read binary BSON data via pymongo
using PyCall
@pyimport pymongo

client = pymongo.MongoClient(addrdict["mongo addr"])
db = client[:CloudGraphs]
collection = "bindata"

IDs = getPoseExVertexNeoIDs(fg.cg.neo4j.connection, session=attrdict["session"], reqbackendset=false);
mongk = getmongokeys(fg, :x1, IDs)
arr = getbinarraymongo(db[collection], mongk["arrnameinneo4j"] )
# arr = getbinarraymongo(db[collection], "58c1a1bdb2ca1534d30b1c96" )





#
