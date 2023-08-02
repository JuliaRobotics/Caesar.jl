

using PyCall
using Images
using ImageFeatures
using DistributedFactorGraphs
using ProgressMeter
using JSON3
using TensorCast
using SHA: sha256

np = pyimport("numpy")
cv = pyimport("cv2")

# # lk_params = ( winSize  = (19, 19), maxLevel = 2, criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
# lk_params = ( winSize  = (19, 19), maxLevel = 2, criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 30, 0.01))
# feature_params = (maxCorners = 1000, qualityLevel = 0.01, minDistance = 8, blockSize = 19 )

function calcFlow(jl_img0, jl_img1, p0, lk_params, back_threshold = 1.0)
    # https://www.programcreek.com/python/example/89363/cv2.calcOpticalFlowPyrLK
    img0 = collect(reinterpret(UInt8, jl_img0))
    img1 = collect(reinterpret(UInt8, jl_img1))
    status = zeros(UInt8, length(p0))
    p1, _st, _err = cv.calcOpticalFlowPyrLK(img0, img1, p0, nothing; lk_params...)
    p0r, _st, _err = cv.calcOpticalFlowPyrLK(img1, img0, p1, nothing; lk_params...)
    d = maximum(reshape(abs.(p0 .- p0r), :, 2); dims=2)#.reshape(-1, 2).max(-1)
    status = d .< back_threshold
    return p1, status
end


function getPose(p1, p2, K)
    # Calculates an essential matrix from the corresponding points in two images.
    E, essmat_mask = cv.findEssentialMat(p1, p2, K, cv.RANSAC, 0.999, 1.0, nothing)
    # Recovers the relative camera rotation and the translation from an estimated 
    # essential matrix and the corresponding points in two images, using chirality check.
    # Returns the number of inliers that pass the check.
    rv, R, t, recpose_mask = cv.recoverPose(E, p1, p2, K, nothing)
    return rv, R, t, recpose_mask
end

function goodFeaturesToTrack(im1, feature_params; mask=nothing) 
    cv.goodFeaturesToTrack(collect(reinterpret(UInt8, im1)), mask=collect(reinterpret(UInt8,mask)), feature_params...)
end

function combinePlot(ref_img, overlay_img)
    combine = map(zip(reinterpret(Gray{N0f8}, ref_img), reinterpret(Gray{N0f8}, overlay_img))) do (a,b)
        RGB(a, b, a)
    end
    f = image(rotr90(combine), axis = (aspect = DataAspect(),))
    return f
end

function getPose(im1, im2, K, feature_params, lk_params; mask=nothing)

    p1 = goodFeaturesToTrack(im1, feature_params; mask)
    
    p2, flow_status = calcFlow(im1, im2, p1, lk_params)
    
    # only keep good points
    p1 = p1[flow_status, :, :]
    p2 = p2[flow_status, :, :]
    
    rv, R, t, recpose_mask = getPose(p1, p2, K)
    return rv, R, t, recpose_mask
end


function trackFeaturesFrames(
  feats0, 
  jl_imgs0_n::AbstractVector{<:AbstractMatrix},
  lk_params;
  mask = nothing
)
  # https://www.programcreek.com/python/example/89363/cv2.calcOpticalFlowPyrLK
  imgs = reinterpret.(UInt8, jl_imgs0_n) .|> collect

  tracks = []
  # status = zeros(UInt8, length(feats0))
  for (i,img) in enumerate(imgs)
    # skip first image which is assumed to coincide with feats0 (good features on first frame)
    i == 1 ? continue : nothing
    p1, _st, _err = cv.calcOpticalFlowPyrLK(imgs[1], img, feats0, nothing; lk_params...) # collect(reinterpret(UInt8,mask))
    push!(tracks, p1)
  end

  return tracks
end


function trackFeaturesForwardsBackwards(imgs, feature_params, lk_params; mask=nothing)

  len = length(imgs)
  @assert isodd(len) "expecting odd number of images for forward backward tracking from center image"

  cen = floor(Int, len/2) + 1
  feats0 = goodFeaturesToTrack(imgs[cen], feature_params; mask)
  # p1 = cv.goodFeaturesToTrack(collect(reinterpret(UInt8, imgs[1])), mask=leftmask, feature_params...)
  # p2, flow_status = calcFlow(imgs[1], imgs[2], p1, lk_params)

  img_tracks = Dict{Int,Vector{Vector{Float64}}}()
  
  img_tracks[0] = [feats0[k,:,:][:] for k in 1:size(feats0,1)]

  tracks = trackFeaturesFrames(feats0, imgs[cen:end], lk_params; mask)
  for (i,tr) in enumerate(tracks)
    img_tracks[i] = [tr[k,:,:][:] for k in 1:size(tr,1)]
  end

  tracks = trackFeaturesFrames(feats0, imgs[cen:-1:1], lk_params; mask)
  for (i,tr) in enumerate(tracks)
    img_tracks[-i] = [tr[k,:,:][:] for k in 1:size(tr,1)]
  end

  return img_tracks
end


function makeBlobFeatureTracksPerImage_FwdBck!(
  dfg::AbstractDFG,
  vlbs_fwdbck::AbstractVector{Symbol},
  imgBlobKey,
  blobstorelbl::Symbol,
  blobLabel::Symbol = Symbol("IMG_FEATURE_TRACKS_FWDBCK_$(length(vlbs_fwdbck))_KLT");
  feature_params,
  lk_params,
  mask,
)
  # good features to track
  kfs = (s->getData(dfg, s, imgBlobKey)).(vlbs_fwdbck)
  imgs = kfs .|> (eb)->unpackBlob(MIME(eb[1].mimeType), eb[2])
  # track features across neighboring frames +-1, +-2, ...
  img_tracks = trackFeaturesForwardsBackwards(imgs, feature_params, lk_params; mask)
  
  center_vlb = vlbs_fwdbck[1+floor(Int,length(vlbs_fwdbck)/2)]

  addDataImgTracksFwdBck!(
    dfg,
    center_vlb,
    blobstorelbl,
    blobLabel,
    "",
    img_tracks,
  )
end


# visualization aid

function plotBlobsImageTracks!(
  dfg::AbstractDFG,
  vlb::Symbol,
  key = r"IMG_FEATURE_TRACKS_FWDBCK";
  fig = GLMakie.Figure(),
  ax = GLMakie.Axis(fig[1,1]),
  resolution::Union{Nothing,<:Tuple} = nothing,
  img::Union{Nothing,<:AbstractMatrix{<:Colorant}} = nothing,
  linewidth = 5
)

  height = 0
  if !isnothing(img)
    image!(ax, rotr90(img))
    height = size(img,1)
  end

  eb = getData(dfg,vlb,key)
  img_tracks = JSON3.read(String(eb[2]), Dict{Int, Vector{Vector{Float32}}})
  
  len = length(img_tracks)
  UU = [Vector{Float64}() for k in 1:len]
  VV = [Vector{Float64}() for k in 1:len]

  fbk = floor(Int, (len-1)/2)
  for k in 1:len
    for i in -fbk:fbk
      push!(UU[k],  img_tracks[i][k][1])  
      push!(VV[k], height-img_tracks[i][k][2])
    end
    lines!(ax, UU[k], VV[k]; color=RGBf(rand(3)...), linewidth)
  end
  if !isnothing(resolution)
    xlims!(ax, 0, resolution[1])
    ylims!(ax, height - resolution[2], height)
    # ylims!(ax, -resolution[2], 0)
  end

  fig
end


