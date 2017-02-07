# find and interact with NEWDATA in neo4j


using Caesar, RoME
using CloudGraphs, Neo4j

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
end


















#
