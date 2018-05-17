# quick test
using Caesar
using Neo4j


include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
addrdict["session"] = "SESSTURT45"  # "SESSROX"
cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)
addrdict["clearslamindb"] = "n"

sessionname = addrdict["session"]
backendset=0
reqbackendset = false
ready=1

cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)
conn = cloudGraph.neo4j.connection

sn = length(sessionname) > 0 ? ":"*sessionname : ""
query = "match (n$(sn):POSE) where n.ready=$(ready) and exists(n.exVertexId)"
query = reqbackendset ? query*" and n.backendset=$(backendset)" : query
query = query*" return n.exVertexId, id(n)"
# query = query*" return n"

@show query

loadtx = transaction(conn)
cph = loadtx(query, submit=true)

ret = Array{Tuple{Int64,Int64},1}()
for data in cph.results[1]["data"]
  exvid, neoid = data["row"][1], data["row"][2]
  push!(ret, (exvid,neoid)  )
end


ret
