# manipulate keyframe images

using Caesar
using CloudGraphs
using Images, ImageMagick
using JSON
using Colors

addrdict = nothing
include(joinpath(dirname(@__FILE__),"..","database","blandauthremote.jl"))
# prepare the factor graph with just one node
# (will prompt on stdin for db credentials)
cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)

session = "SESSTURT21"
frd = fetchrobotdatafirstpose(cloudGraph, session)
camf = frd["CAMK"][1,1]
camc = frd["CAMK"][1,3]
tagwidth = 0.172

dcamjl = DepthCamera(frd["CAMK"])
buildmesh!(dcamjl)


# convenience quick peek function
@async cloudimshow(cloudGraph, session, :x15)


vsym = :x15


cv = getCloudVert(cloudGraph, session, vsym, bigdata=true);
imdata = Caesar.getBigDataElement(cv, "keyframe_rgb").data;
img = ImageMagick.readblob(imdata);


imshowhackpng(imdata)


neis = CloudGraphs.get_neighbors(cloudGraph, cv)
neis[2].properties["label"]
jd = JSON.parse(neis[2].properties["frtend"])
meas = parse.(split(jd["meas"]))


atbeari = meas[1]
atdepth = meas[2]*520/570
deptstd = meas[5]
measimgcol = round(Int,camc-camf*tan(atbeari)) # measurement center
atvertsides = tagwidth/2/atdepth*camf

r, c = size(img)
for i in 1:r
  img[i, measimgcol] = RGB(1.0,0.0,0.0)
  side1 = round(Int,measimgcol-atvertsides)
  side2 = round(Int,measimgcol+atvertsides)
  if 0 < side1 <= c
    img[i, side1] = RGB(1.0,1.0,0.0)
  end
  if 0 < side2 <= c
    img[i, side2] = RGB(1.0,1.0,0.0)
  end
end


filename = joinpath("/tmp","testimg.png")
ImageMagick.save_(filename, img)

@async run(`eog $(filename)`)



# Look at xtion depth information




depthdata = Caesar.getBigDataElement(cv, "depthframe_image").data;
arr = bin2arr(depthdata, dtype=Float32) # should also store dtype for arr in Mongo
depthimg = Array{Float64,2}(reshape(arr, c, r)')

depthcloud = reconstruct(dcamjl, depthimg)

# using Gadfly


plot(
  layer(y=depthcloud[:,measimgcol,3],Geom.line),
  layer(y=depthcloud[:,measimgcol,3]+deptstd,Geom.line),
  layer(y=depthcloud[:,measimgcol,3]-deptstd,Geom.line),
  Guide.title("AprilTag says: $(atdepth)")
)


jd


# SLAM estimates of distance
using IncrementalInference, KernelDensityEstimate

pvert = cloudVertex2ExVertex(cv)
lsym = :l200578
lcv = getCloudVert(cloudGraph, session, lsym, bigdata=false);
lvert = cloudVertex2ExVertex(lcv)


posepos = getKDEMax(getVertKDE(pvert))
landpos = getKDEMax(getVertKDE(lvert))

norm(posepos[1:2]-landpos[1:2])



#
