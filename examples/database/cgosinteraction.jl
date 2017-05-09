# template log-on and fetch graph

using Caesar


include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
session = "SESSHAUVDEV2"
addrdict["session"] = session
cg, addrdict = standardcloudgraphsetup(addrdict=addrdict)



@time all = ls(cg, "SESSTURT21")


@time dd = ls(cg, session, sym=:x1)

dd2 = ls(cg, session, exvid=5)



findAllBinaryFactors(cg, session)



using IncrementalInference


fsym = collect(keys(dd))[2]
cv = getCloudVert(cg, session, sym=fsym)

ty = getfnctype(cv)

fieldnames(ty.name)

@show ty.name.name







#
