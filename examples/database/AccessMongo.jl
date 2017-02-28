using Mongo, LibBSON
using Caesar

## Uncomment out for command line operation
cloudGraph, addrdict = standardcloudgraphsetup()

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
mongoClient = cloudGraph.mongo.client
# mongoTestCollection = MongoCollection(mongoClient, "tester", "testing_collection")
# cursor = find(mongoTestCollection, query())
# for o in cursor
#     println("Key: ", o["key"], ", oid: ", o["_id"])  # Or simply,
#     #println(o)
# end
