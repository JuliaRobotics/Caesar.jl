# Julia calling python for vizualization in MIT colletions viewer.
# We should converge with Director, but need the collections renderer code
# in Director before that will happen
#
# This code uses Sudeep's Python drawing utils and transformation code

using Caesar, RoME, KernelDensityEstimate
using IncrementalInference, CloudGraphs, Neo4j


# dbaddress = "mrg-liljon.csail.mit.edu"

#
# # connect to the server, CloudGraph stuff
dbaddress = length(ARGS) > 0 ? ARGS[1] : "localhost"
println("Taking Neo4j database address as $(dbaddress)...")

# switch IncrementalInference to use CloudGraphs (Neo4j) data layer
configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, "", "", dbaddress, 27017, false, "", "");
cloudGraph = connect(configuration);
IncrementalInference.setCloudDataLayerAPI!()
# Connection to database for additional queries
conn = cloudGraph.neo4j.connection

# Register protobuf types of interest in CloudGraphs layer
registerGeneralVariableTypes!(cloudGraph)

using PyCall

@pyimport pybot.geometry.rigid_transform as bgrt
@pyimport pybot.geometry.quaternion as bgq

# usage example
#	bgq.Quaternion[:from_rpy](0,0,0)

# this resets the Collections Viewer visualization
@pyimport pybot.externals.lcm.draw_utils as bedu
@pyimport pybot.vision.camera_utils as cu



@pyimport numpy as np
@pyimport cv2 as opencv

# draw_utils.publish_pose_list('apriltag', self.tagwposes, frame_id='origin', texts=ids)


# CAMK [[ 570.34222412    0.          319.5       ]
#  [   0.          570.34222412  239.5       ]
#  [   0.            0.            1.        ]]

CAMK = [[ 570.34222412; 0.0; 319.5]';
 [   0.0; 570.34222412; 239.5]';
 [   0.0; 0.0; 1.0]'];

dcam = cu.DepthCamera(K=CAMK)

poseswithdepth = Dict()
poseswithdepth["x1"] = 0 # skip this pose -- there is no big data before ICRA

while true
  # this is being replaced by cloudGraph, added here for development period
  fg = emptyFactorGraph()
  fg.cg = cloudGraph

    IDs = getPoseExVertexNeoIDs(conn);

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
			# Xpp,Ypp, Thpp, LBLS = get2DPoseMax(fg)
			poses = []
			# for i in 1:length(Xpp)
			for x in xx
				# posei = bgrt.RigidTransform[:from_angle_axis](Thpp[i],[0;0;1],[Xpp[i];Ypp[i];0])
				vert = getVert(fg,x)
				X = Float64[]
				if haskey(vert.attributes, "MAP_est")
					X = vert.attributes["MAP_est"]
				else
					X = getKDEMax(getKDE(vert))
				end
				posei = bgrt.RigidTransform[:from_angle_axis](X[3],[0;0;1],[X[1];X[2];0])
				push!(poses, posei)
			end
			bedu.publish_pose_list("MAPposes",poses, frame_id="origin",texts=xx)

      for x in xx
  			if !haskey(poseswithdepth, x)
          poseswithdepth[x]=1
          cvid = -1
          for id in IDs
            if fg.g.vertices[id[1]].label == x
              cvid = id[2]
              break
            end
          end
          # fetch depth cloud from mongo for pose
          cv = CloudGraphs.get_vertex(fg.cg, cvid)
          bd = CloudGraphs.read_BigData!(cloudGraph, cv)
          depthcloudpng = nothing
          for i in bd.dataElements
            if i.description == "depthframe-image"
              depthcloudpng = i.data
              break
            end
          end
          #interpret data
          fid = open("tempdepth.png","w")
          write(fid, depthcloudpng)
          close(fid)
          img = opencv.imread("tempdepth.png",2)
          imgc = map(Float64,img)/1000.0
          # for i in 1:100
          #   opencv.imshow("yes", img)
          #   opencv.waitKey(1)
          # end
          # calibrate the image
          # color conversion of points, so we can get pretty pictures...
          X = dcam[:reconstruct](imgc)
          # get color information
          rgbpng = nothing
          for i in bd.dataElements
            if i.description == "keyframe-image"
              rgbpng = i.data
              break
            end
          end
          #interpret data
          fid = open("temprgb.png","w")
          write(fid, rgbpng)
          close(fid)
          rgb = opencv.imread("temprgb.png")
          # cv2.cvtColor(im,im,cv2.COLOR_BGR2RGB)
          # rgb = bgr[:,:,[3;2;1]]
          # publish to viewer via Sudeeps python code
          bedu.publish_cloud("depth", X, c=rgb, frame_id="camera", flip_rb=true)
        end
      end

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
