

using Flux, RoME, RoMEPlotting
using ImageMagick, Images, ImageView, ImageDraw
using Colors
using JSON2

using FreeTypeAbstraction, AprilTags


function imdrawline!(img, col; color=RGB(1.0,0.0,0.0) )
  for i in 1:size(img, 1)
    img[i, col] = color
  end
end



cd("/media/dehann/logs3/caesar/2020-08-19T11:45:38.074")

fg = loadDFG("fg_final_resolve.tar.gz")

# wire up the blob store
getSolverParams(fg).logpath = pwd()
storeDir = joinLogPath(fg,"data")
mkpath(storeDir)
datastore = FolderStore{Vector{UInt8}}(:default_folder_store, storeDir) 
addBlobStore!(fg, datastore)


# Retrieve camera calibration
listDataEntries(fg, :x0)
rD = getData(fg, :x0, :camCalib)
camCalib = JSON2.read(IOBuffer(rD[2]))
K = reshape(camCalib[:vecK], camCalib[:size]...) .|> Float64


# get image from x1
imgE ,imgD = getData(fg, :x1, :KEYFRAME_IMG)
img = ImageMagick.readblob(imgD);
# check that we have the image
imshow(img)


# detect and draw tags
detector = AprilTagDetector()
tags = detector(img)



img_ = drawTags(img,K, tags)
imshow(img_)


ls(fg, :x1)

x1l2f1 = getFactorType(fg, :x1l2f1)

x1l2f1.z.μ


function calcBearingRange(dfg::AbstractDFG, from::Symbol, to::Symbol)
  getSofttype(dfg, from)
end



measimgcol = round(Int,camc-camf*tan(getfnctype(ve).bearing.μ)) # measurement center
imdrawline!(img, measimgcol)


# deptdist = Base.mean(depthcloud[260:270,measimgcol,3])
# push!(DISTS[fct], deptdist)



