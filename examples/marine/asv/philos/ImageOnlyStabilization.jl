
function transformIndexes(bP__::AbstractVector{C}, aTb) where {C <: CartesianIndex{2}}
  bV = zeros(3)
  bV[3] = 1.0

  aV = zeros(3)
  aP__ = Vector{Vector{Float64}}(undef, length(bP__))

  for (i,kp) in enumerate(bP__)
    bV[1] = kp[1]
    bV[2] = kp[2]

    aV[:] = aTb*bV
    aP__[i] = aV[1:2]
  end

  return aP__
end


function cost(matches, aTb_)
  pts1 = (x->x[1]).(matches) .|> x-> [x[1];x[2]]
  _pts2 = (x->x[2]).(matches)
  pts2 = transformIndexes(_pts2, aTb_)

  total = 0.0
  for (i,p2) in enumerate(pts2)
    total += norm( pts1[i] .- p2 )
  end

  return total
end

function cost_xyr!(xyr, matches, aTb = [1. 0 0; 0 1 0; 0 0 1])
  aTb[1:2,1:2] .= _Rot.RotMatrix(xyr[3])
  aTb[1:2, 3] .= xyr[1:2]
  cost(matches, aTb)
end

# draw matches

function mosaicMatches(img1, img2, matches)
  grid = hcat(img1, img2)
  offset = CartesianIndex(0, size(img1, 2))
  map(m -> draw!(grid, LineSegment(m[1], m[2] + offset)), matches)
  grid
end

function getFeaturesDescriptors(img::AbstractMatrix)
  # img = parent(img_)
  brisk_params = BRISK()
  features = Features(Keypoints(imcorner(img, method=harris)))
  desc, ret_features = create_descriptor(Gray.(img), features, brisk_params)
end

function getCornersDescriptors( img::AbstractMatrix; 
                                kparg1 = 15, kparg2=0.3,
                                size = 256, window = 20, seed = 123 )
  #
  brief_params = BRIEF(;size, window, seed)
  keypoints = Keypoints(fastcorners(img, kparg1, kparg2))
  desc, ret_features = create_descriptor(img, keypoints, brief_params);
end

function getKeypointsMatchBRIEF(  img1, img2; 
                                  kparg1 = 15, kparg2=0.3, threshold=0.1,
                                  size = 256, window = 20, seed = 123 )
  #
  desc_1, ret_kps_1 = getCornersDescriptors( img1; kparg1, kparg2, size, window, seed )
  desc_2, ret_kps_2 = getCornersDescriptors( img2; kparg1, kparg2, size, window, seed )

  match_keypoints(ret_kps_1, ret_kps_2, desc_1, desc_2, threshold)
end

function getKeypointsMatchBRISK(img1, img2; 
                                threshold=0.1,
                                kwargs... )
  #
  desc_1, ret_kps_1 = getFeaturesDescriptors( img1; kwargs... )
  desc_2, ret_kps_2 = getFeaturesDescriptors( img2; kwargs... )

  match_keypoints(Keypoints(ret_kps_1), Keypoints(ret_kps_2), desc_1, desc_2, threshold)
end


function calcMatchMinimize( img1::AbstractMatrix{<:Gray}, img2::AbstractMatrix{<:Gray};
                            keypoint_fnc::Function=getKeypointsMatchBRIEF,
                            kwargs...)
  #
  
  matches = keypoint_fnc( img1, img2; kwargs... )

  res = Optim.optimize((xyr)->cost_xyr!(xyr, matches), [0.0;0.0;0.0])
  res.minimizer, matches
end


function pushMatchWindowMedian!(iCB::AbstractVector{<:AbstractMatrix{<:Gray}}, 
                                trBuffer::AbstractVector{<:AbstractVector},
                                newImg::AbstractMatrix{<:Gray};
                                kwargs... )
  #
  for bimg in iCB
    trans, matches = calcMatchMinimize(newImg, bimg; kwargs...);
    push!(trBuffer, trans)
  end

  Ti = median((x->x[1]).(trBuffer))
  Tj = median((x->x[2]).(trBuffer))
  Tr = median((x->x[3]).(trBuffer))

  return [Ti; Tj; Tr]
end

Base.@kwdef struct BasicImageOnlyStabilization{N}
  imgBuf::CircularBuffer{<:AbstractMatrix{<:Gray}} = CircularBuffer{Matrix{Gray{N0f8}}}(N)
  trBuf::CircularBuffer{<:AbstractVector{Float64}} = CircularBuffer{Vector{Float64}}(N)
end

function (bi::BasicImageOnlyStabilization)( newImg::AbstractMatrix;
                                            offset::AbstractVector=zeros(3),
                                            roi_i = :,
                                            roi_j = :,
                                            scale_i=1,
                                            scale_j=1,
                                            scale_r=1,
                                            kwargs... )
  #
  img_g = Gray.(newImg[roi_i,roi_j])
  if length(bi.imgBuf) == 0
    push!(bi.imgBuf, img_g)
  end
  tr = pushMatchWindowMedian!(bi.imgBuf, bi.trBuf, img_g ; kwargs... )
  rot = ImageTransformations.recenter(_Rot.RotMatrix(scale_r*tr[3] + offset[3]), [Base.size(img_g)...] .÷ 2)  # a rotation around the center
  
  tform = rot ∘ Translation(scale_i*tr[1],scale_j*tr[2])
  warp(newImg, tform, axes(newImg))
end





#