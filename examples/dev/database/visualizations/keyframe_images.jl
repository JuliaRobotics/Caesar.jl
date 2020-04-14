# manipulate keyframe images

using Caesar
using CloudGraphs
using Images, ImageMagick
using JSON
using Colors
using Gadfly
using IncrementalInference, KernelDensityEstimate


addrdict = nothing
include(joinpath(dirname(@__FILE__),"..","database","blandauthremote.jl"))
# prepare the factor graph with just one node
# (will prompt on stdin for db credentials)
cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)

session = "SESSTURT38"
frd = fetchrobotdatafirstpose(cloudGraph, session)
camf = frd["CAMK"][1,1]
camc = frd["CAMK"][1,3]
tagwidth = 0.172

dcamjl = DepthCamera(frd["CAMK"])
buildmesh!(dcamjl)




# convenience quick peek function
@async cloudimshow(cloudGraph, session, :x3)





# prematurely get subgraph around all landmarks
ids = getExVertexNeoIDs(cloudGraph.neo4j.connection, label="LANDMARK", session=session)

sfg = initfg(cloudgraph=cloudGraph, sessionname=session)
fetchsubgraph!(sfg, Int[ids[i][2] for i in 1:length(ids)], numneighbors=2 )

xx,ll = ls(sfg)



vsym = xx[5]  #:x19


# fetch and show images
cv = getCloudVert(cloudGraph, session, sym=vsym, bigdata=true);
sleep(0.5)
imdata = Caesar.getBigDataElement(cv, "keyframe_rgb").data;
img = ImageMagick.readblob(imdata);

@async imshowhackpng(imdata)
@async imshowhack(img)  # prefered




neis = CloudGraphs.get_neighbors(cloudGraph, cv)

neis[2].properties["label"]
jd = JSON.parse(neis[2].properties["frtend"])
meas = parse.(split(jd["meas"]))

jd


atbeari = meas[1]
atdepth = meas[2]#*520/570
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


# get slam vertex elements
lms = landmarks(sfg, vsym)
@show lsym = lms[2]
# SLAM estimates of distance
slamdist = getRangeKDEMax2D(cloudGraph, session, vsym, lsym)

deptdist = Base.mean(depthcloud[260:270,measimgcol,3])

plot(
  layer(y=depthcloud[:,measimgcol,3],Geom.line),
  # layer(y=depthcloud[:,measimgcol,3]+deptstd,Geom.line),
  # layer(y=depthcloud[:,measimgcol,3]-deptstd,Geom.line),
  Guide.title("AprilTag says: $(round(atdepth,3))\n
  SLAM says is: $(round(slamdist,3))\n
  Depth cam is: $(round(deptdist,3))\n
  AprilTag id:$(lsym)")
)









# investigate belief on vertices
ls(sfg, lsym)
vsym
plotKDEresiduals(sfg, :x105l200545, api=localapi)
getVal(sfg, lsym, api=localapi)
plotKDE(sfg, lsym, dims=[1;2], api=localapi)


pro, = localProduct(sfg, lsym)
plotKDE(pro)
lvert = getVert(sfg, lsym, api=localapi)
setVal!(lvert, getPoints(pro), getBW(pro)[:,1])
plotKDE(getVertKDE(lvert))

plotLocalProduct(sfg, lsym, api=localapi)

# via full graph copy
# fg = initfg(cloudgraph=cloudGraph, sessionname=session)
# fullLocalGraphCopy!(fg)






# for l in ll
fsym = ls(sfg, ll[1])
fs = fsym[1]
if length(lsf(sfg, fs)) > 1
  @show fs
  ty = getfnctype(sfg, sfg.fIDs[fs])
  zij = ty.range.μ
  pij = getRangeKDEMax2D(sfg, ll[1])
end

# end
#
