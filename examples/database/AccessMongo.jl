using Mongo, LibBSON
using Caesar

#Insert your Cloudgraphs config here so you produce a configuration as below
#...
include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, dbusr, dbpwd, mongoaddress, 27017, false, "", "");

cloudGraph = connect(configuration);
conn = cloudGraph.neo4j.connection
registerGeneralVariableTypes!(cloudGraph)

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
