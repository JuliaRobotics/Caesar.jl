# Julia calling python for vizualization in MIT colletions viewer.
# We should converge with Director, but need the collections renderer code
# in Director before that will happen
#
# This code uses Sudeep's Python drawing utils and transformation code

using Caesar, RoME
# using CloudGraphs
using IncrementalInference, CloudGraphs, Neo4j

# switch IncrementalInference to use CloudGraphs (Neo4j) data layer

# connect to the server, CloudGraph stuff
dbaddress = length(ARGS) > 0 ? ARGS[1] : "localhost"
println("Taking Neo4j database address as $(dbaddress)...")
configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, "", "", "localhost", 27017, false, "", "");
cloudGraph = connect(configuration);
# register types of interest in CloudGraphs
registerGeneralVariableTypes!(cloudGraph)
IncrementalInference.setCloudDataLayerAPI!()

# Connect to database
conn = cloudGraph.neo4j.connection


using PyCall

@pyimport bot_geometry.rigid_transform as bgrt
@pyimport bot_geometry.quaternion as bgq

# usage example
#	bgq.Quaternion[:from_roll_pitch_yaw](0,0,0)

# this resets the Collections Viewer visualization
@pyimport bot_externals.draw_utils as bedu

# draw_utils.publish_pose_list('apriltag', self.tagwposes, frame_id='origin', texts=ids)



while true
  # this is being replaced by cloudGraph, added here for development period
  println("=================================================")
  fg = emptyFactorGraph()
  fg.cg = cloudGraph

    println("get local copy of graph")
    if fullLocalGraphCopy!(fg, conn)
			xx,ll = ls(fg)
			LD = Array{Array{Float64,1},1}()
			C = Vector{ASCIIString}()
			for x in xx
				val = getVal(fg,x)
				len = size(val,2)
				[push!(C,"g") for i in 1:len];
				[push!(LD, vec([val[1:2,i];0])) for i in 1:len];
			end
			bedu.publish_cloud("posePts", LD,c=C, frame_id="origin")
			# draw pose MAP triads
			Xpp,Ypp, Thpp, LBLS = get2DPoseMax(fg)
			poses = []
			for i in 1:length(Xpp)
				posei = bgrt.RigidTransform[:from_angle_axis](Thpp[i],[0;0;1],[Xpp[i];Ypp[i];0])
				push!(poses, posei)
			end
			bedu.publish_pose_list("MAPposes",poses, frame_id="origin",texts=LBLS)
			# now do landmarks
			LD = Array{Array{Float64,1},1}()
			C = Vector{ASCIIString}()
			for l in ll
				val = getVal(fg,l)
				len = size(val,2)
				[push!(C,"c") for i in 1:len];
				[push!(LD, vec([val[1:2,i];0])) for i in 1:len];
			end
			bedu.publish_cloud("landmarkPts", LD,c=C, frame_id="origin")
    else
      sleep(0.2)
    end
		sleep(0.05)
end
#
