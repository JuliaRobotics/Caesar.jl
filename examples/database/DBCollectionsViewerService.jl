# Julia calling python for vizualization in MIT colletions viewer.
# We should converge with Director, but need the collections renderer code
# in Director before that will happen
#
# This code uses Sudeep's Python drawing utils and transformation code

using Caesar, RoME, KernelDensityEstimate
using IncrementalInference, CloudGraphs, Neo4j
using TransformUtils

using PyCall



# Uncomment out for command line operation
cloudGraph, addrdict = standardcloudgraphsetup(drawdepth=true)
session = addrdict["session"]
DRAWDEPTH = addrdict["draw depth"]=="y" || addrdict["draw depth"]=="yes"

mongoaddress = addrdict["mongo addr"]
collection = "bindata"


# # interactive operation
# session = "SESSTURT21"
# Nparticles = 100
# include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
# DRAWDEPTH = true

# include("VisualizationUtilities.jl")  # @pyimport getimages as gi


@pyimport pybot.geometry.rigid_transform as bgrt
@pyimport pybot.geometry.quaternion as bgq

# usage example
#	bgq.Quaternion[:from_rpy](0,0,0)

# this resets the Collections Viewer visualization
@pyimport pybot.externals.lcm.draw_utils as bedu
# @pyimport pybot.vision.camera_utils as cu


@pyimport numpy as np
@pyimport cv2 as opencv


# draw_utils.publish_pose_list('apriltag', self.tagwposes, frame_id='origin', texts=ids)

# CAMK [[ 570.34222412    0.          319.5       ]
#  [   0.          570.34222412  239.5       ]
#  [   0.            0.            1.        ]]

CAMK = [[ 570.34222412; 0.0; 319.5]';
 [   0.0; 570.34222412; 239.5]';
 [   0.0; 0.0; 1.0]'];

# dcam = cu.DepthCamera(K=CAMK)
# -np.pi/2, 0, -np.pi/2
dcamjl = DepthCamera(CAMK)
buildmesh!(dcamjl)

temp = bgrt.RigidTransform[:from_rpyxyz](-pi/2, 0, -pi/2, 0, 0, 0.6, axes="sxyz")
# @show temp[:to_matrix]()

# bTc = SE3([0;0;0.6],AngleAxis(-pi/2,[1;0;0])*AngleAxis(-pi/2,[0;0;1]))
bTc = SE3(temp[:to_matrix]())



poseswithdepth = Dict()
poseswithdepth["x1"] = 0 # skip this pose -- there is no big data before ICRA
# poseswithdepth["x52"] = 0 # skip this pose -- there is no big data before ICRA
# poseswithdepth["x53"] = 0 # skip this pose -- there is no big data before ICRA
# poseswithdepth["x54"] = 0 # skip this pose -- there is no big data before ICRA
# poseswithdepth["x55"] = 0 # skip this pose -- there is no big data before ICRA



while true

  # this is being replaced by cloudGraph, added here for development period
  fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)

  IDs = getPoseExVertexNeoIDs(fg.cg.neo4j.connection, session=session, reqbackendset=false);

  println("get local copy of graph")
  if fullLocalGraphCopy!(fg, reqbackendset=false)
  	xx,ll = ls(fg)
  	LD = Array{Array{Float64,1},1}()
  	C = Vector{AbstractString}()
  	for x in xx
  		val = getVal(fg, x)
  		len = size(val,2)
  		[push!(C,"g") for i in 1:len];
  		[push!(LD, vec([val[1:2,i];0])) for i in 1:len];
  	end
  	bedu.publish_cloud("posePts", LD,c=C, frame_id="origin")
  	# draw pose MAP triads
  	# Xpp,Ypp, Thpp, LBLS = get2DPoseMax(fg)
  	poses = []
    cams = []
  	# for i in 1:length(Xpp)
    j=0
  	for x in xx
      j+=1
  		# posei = bgrt.RigidTransform[:from_angle_axis](Thpp[i],[0;0;1],[Xpp[i];Ypp[i];0])
  		vert = getVert(fg,x, api=localapi)
  		X = Float64[]
  		if haskey(vert.attributes, "MAP_est")
  			X = vert.attributes["MAP_est"]
  		else
  			X = getKDEfit(getKDE(vert))
  		end
  		posei = bgrt.RigidTransform[:from_angle_axis](X[3],[0;0;1],[X[1];X[2];0])
  		push!(poses, posei)
      wTb = SE3([X[1];X[2];0], TransformUtils.AngleAxis(X[3],[0;0;1]))
      wTc = wTb*bTc
      camiRT = bgrt.RigidTransform[:from_matrix](matrix(wTc))
      cami = bgrt.Pose[:from_rigid_transform](j, camiRT)

      push!(cams, cami)
  	end
  	bedu.publish_pose_list("MAPposes",poses, frame_id="origin",texts=xx)
    bedu.publish_pose_list("MAPcams",cams, frame_id="origin",reset=false)

    j=0
    for x in xx
      j+=1
      mongk, cvid = getmongokeys(fg, x, IDs)
  		if DRAWDEPTH && haskey(mongk, "depthframe_image") # !haskey(poseswithdepth, x)
        poseswithdepth[x]=1
        mongo_keydepth = bson.ObjectId(mongk["depthframe_image"])
        img, ims = gi.fastdepthimg(db[collection], mongo_keydepth)
        # cvid = -1
        # for id in IDs
        #   if Symbol(fg.g.vertices[id[1]].label) == x
        #     cvid = id[2]
        #     break
        #   end
        # end
        # # fetch depth cloud from mongo for pose
        # cv = CloudGraphs.get_vertex(fg.cg, cvid)
        # bd = CloudGraphs.read_BigData!(cloudGraph, cv)
        # depthcloudpng = nothing
        # for i in bd.dataElements
        #   if i.description == "depthframe-image"
        #     depthcloudpng = i.data
        #     break
        #   end
        # end
        # #interpret data
        # fid = open("tempdepth.png","w")
        # write(fid, depthcloudpng)
        # close(fid)
        # img = opencv.imread("tempdepth.png",2)
        # imgc = map(Float64,img)/1000.0
        #   opencv.imshow("yes", img)
        # calibrate the image # color conversion of points, so we can get pretty pictures...
        # X = dcam[:reconstruct](img)
        X = reconstruct(dcamjl, Array{Float64,2}(img))
        r,c,h = size(X)
        Xd = X[1:3:r,1:3:c,:]
        mask = Xd[:,:,:] .> 4.5
        Xd[mask] = Inf
        # get color information
        rgb = nothing
        seg = nothing
        if haskey(mongk, "keyframe_rgb")
          mongo_key = bson.ObjectId(mongk["keyframe_rgb"])
          rgb, ims = gi.fastrgbimg(db[collection], mongo_key)
          # @show size(rgb)
        end
        if haskey(mongk, "keyframe_segnet")
          mongo_key = bson.ObjectId(mongk["keyframe_segnet"])
          seg, ims = gi.fastrgbimg(db[collection], mongo_key)
        end
        # for i in bd.dataElements
        #   if i.description == "keyframe-image"
        #     rgbpng = i.data
        #   end
        #   if i.description == "segnet-image"
        #     segpng = i.data
        #   end
        # end
        #interpret data
        if rgb != nothing
          # fid = open("temprgb.png","w")
          # write(fid, rgbpng)
          # close(fid)
          # rgb = opencv.imread("temprgb.png")
          # rgb = bgr[:,:,[3;2;1]]
          # publish to viewer via Sudeeps python code
          # @show size(rgb), typeof(rgb)
          rgbss = rgb[1:3:r,1:3:c,:]
          bedu.publish_cloud("depth", Xd, c=rgbss, frame_id="MAPcams",element_id=j, flip_rb=true, reset=false)
        end
        if seg != nothing
          # fid = open("temprgb.png","w")
          # write(fid, segpng)
          # close(fid)
          # seg = opencv.imread("temprgb.png")
          # @show size(seg), typeof(seg)
          segss = seg[1:3:r,1:3:c,:]
          bedu.publish_cloud("segnet", Xd, c=segss, frame_id="MAPcams",element_id=j, flip_rb=true, reset=false)
        end
      end
    end

  	# now do landmarks
  	LD = Array{Array{Float64,1},1}()
  	C = Vector{AbstractString}()
    landms = []
  	for l in ll
      vert = getVert(fg,l, api=localapi)

  		val = getVal(vert)
  		len = size(val,2)
  		[push!(C,"c") for i in 1:len];
  		[push!(LD, vec([val[1:2,i];0])) for i in 1:len];

  		X = Float64[]
  		if haskey(vert.attributes, "MAP_est")
  			X = vert.attributes["MAP_est"]
  		else
  			X = getKDEMax(getKDE(vert))
  		end

      lmi = bgrt.RigidTransform[:from_angle_axis](0,[0;0;1],[X[1];X[2];0])
  		push!(landms, lmi)

  	end
  	bedu.publish_cloud("landmarkPts", LD,c=C, frame_id="origin")
    bedu.publish_pose_list("MAPlandms",landms, frame_id="origin",texts=ll)
  else
    sleep(0.2)
  end
  sleep(0.05)

end












#
