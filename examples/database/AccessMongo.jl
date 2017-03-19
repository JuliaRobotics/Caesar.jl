using Mongo, LibBSON
using Caesar

include(joinpath(dirname(@__FILE__),"..","database","blandauthremote.jl"))
cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)
# cloudGraph, addrdict = standardcloudgraphsetup()

## interactive operation
# include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))

# Get the keys (and possibly the data)
cursor = find(cloudGraph.mongo.cgBindataCollection, query())
for o in cursor
    # println( ", oid: ", o["_id"])
    println("id: ", o["_id"]);
end

# Get Roxana's encoded parameters
#REF: https://github.com/dehann/Caesar.jl/blob/master/examples/database/python/mongo_interaction.py
# mongoClient = cloudGraph.mongo.client
# mongoTestCollection = MongoCollection(mongoClient, "tester", "testing_collection")
# cursor = find(mongoTestCollection, query())
# for o in cursor
#     println("Key: ", o["key"], ", oid: ", o["_id"])  # Or simply,
#     #println(o)
# end



# fetch a binary png image
myKeyToFind = BSONOID("58af67255d7625647859fa71")
findsomthing = find(cloudGraph.mongo.cgBindataCollection, ("_id" => eq(myKeyToFind)))
# @show myFavouriteKey["val"]
myFavouriteKey = first( findsomthing );

file = open("test.png", "w")
write(file, myFavouriteKey["val"])
close(file)
run(`eog test.png`)
run(`rm test.png`)




# limitations in LibBSON.jl prevent Binary unpacking at this time, using PyCall to pymongo interim
# key = BSONOID("58c1a1bdb2ca1534d30b1c96")
# obj = find(cloudGraph.mongo.cgBindataCollection, ( "_id" => eq(key) ) )

# fetching data from Mongo using getimages.py
# IDs = getPoseExVertexNeoIDs(fg.cg.neo4j.connection, sessionname=attrdict["session"], reqbackendset=false);
# mongk = getmongokeys(fg, :x1, IDs)
# mongo_key = bson.ObjectId(mongk["keyframe_rgb"])
# arr = gi.getbinarray(dbcoll, mongo_key)







# temporary helper function to read binary BSON data via pymongo
using PyCall
@pyimport pymongo

client = pymongo.MongoClient(addrdict["mongo addr"])
db = client[:CloudGraphs]
collection = "bindata"

IDs = getPoseExVertexNeoIDs(fg.cg.neo4j.connection, sessionname=attrdict["session"], reqbackendset=false);
mongk = getmongokeys(fg, :x1, IDs)
arr = getbinarraymongo(db[collection], mongk["arrnameinneo4j"] )
# arr = getbinarraymongo(db[collection], "58c1a1bdb2ca1534d30b1c96" )





#
