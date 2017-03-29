# Simple fetch single image from MongoDB store, using Neo4j lookup

using Caesar

# include(joinpath(dirname(@__FILE__),"..","database","blandauthremote.jl"))
# addrdict["session"] = "SESSTURT45"
# cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)
cloudGraph, addrdict = standardcloudgraphsetup()


session = addrdict["session"]

@show IDs = getPoseExVertexNeoIDs(
  cloudGraph.neo4j.connection,
  sessionname=session,
  reqbackendset=false
);

vid, cvid = IDs[2]

# cv = CloudGraphs.get_vertex(cloudGraph, cvid, false )
cv = CloudGraphs.get_vertex(cloudGraph, cvid, true  )

img = Caesar.getBigDataElement(cv, "keyframe_rgb")
imf = open("test.png","w")
write(imf, img.data)
close(imf)
run(`eog test.png`)
Base.rm("test.png")

rgb = ImageMagick.readblob(img.data);



# using JSON
# using LibBSON
#
# mongk = JSON.parse(cv.properties["mongo_keys"])
# BSONOID(mongk["keyframe_rgb"])







#
