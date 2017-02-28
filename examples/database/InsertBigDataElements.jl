# Insert stuff into mongo


using Caesar
using IncrementalInference, CloudGraphs, Neo4j



## Uncomment out for command line operation
# cloudGraph, addrdict = standardcloudgraphsetup(nparticles=true)
# session = addrdict["session"]
# Nparticles = parse(Int,addrdict["num particles"])

# interactive operation
session = "SESSROX"
Nparticles = 100
include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))


#
# if fullLocalGraphCopy!(fg, conn)
# 	error("could not get full graph")
# end

IDs = getPoseExVertexNeoIDs(conn)

# i=2
# for i in 3:length(IDs)
# 	cv = CloudGraphs.get_vertex(cloudGraph, IDs[i][2])
#
# 	fid = open("/home/dehann/software/pyslam-pod/examples/images/keyframe_x$(i).png","r")
# 	imageData = readbytes(fid) # imageData::Vector{UInt8}
# 	close(fid)
# 	bdei = CloudGraphs.BigDataElement("keyframe-image", imageData)
# 	push!(cv.bigData.dataElements, bdei);
#
# 	fid = open("/home/dehann/software/pyslam-pod/examples/images/depthframe_x$(i).png","r")
# 	imageData = readbytes(fid) # imageData::Vector{UInt8}
# 	close(fid)
#
# 	bded = CloudGraphs.BigDataElement("depthframe-image", imageData)
# 	push!(cv.bigData.dataElements, bded);
#
# 	CloudGraphs.save_BigData!(cloudGraph, cv)
# end

aa = Int64[]
for i in 3:181
	push!(aa, IDs[i][1])

end



@show diffaa

i=1
for i in 3:181
	cv = CloudGraphs.get_vertex(cloudGraph, IDs[i][2])

	bd = CloudGraphs.read_BigData!(cloudGraph, cv)

	fid = open("/home/dehann/.julia/v0.4/Caesar/imagesnames/segnet/segnet-labels/colored_x$(i).png","r")
	imageData = readbytes(fid) # imageData::Vector{UInt8}
	close(fid)
	bdei = CloudGraphs.BigDataElement("segnet-image", imageData)
	push!(cv.bigData.dataElements, bdei);

	CloudGraphs.save_BigData!(cloudGraph, cv)
end


for j in  3:length(IDs)
	@show j
	cv = CloudGraphs.get_vertex(cloudGraph, IDs[j][2])
	bd = CloudGraphs.read_BigData!(cloudGraph, cv)
	depthcloudpng = nothing
	for i in bd.dataElements
		@show i.description
		if i.description == "keyframe-image"
			depthcloudpng = i.data
			break
		end
	end
	#interpret data
	fid = open("/home/dehann/.julia/v0.4/Caesar/imagesnames/keyframe_x$(j).png","w")
	write(fid, depthcloudpng)
	close(fid)

end












#
