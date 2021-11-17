
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

# aTb = [1. 0 0; 0 1 0; 0 0 1]
cost_tr(xy, matches) = cost(matches, [1. 0 xy[1]; 0 1 xy[2]; 0 0 1])

# draw matches

function mosaicMatches(img1, img2, matches)
  grid = hcat(img1, img2)
  offset = CartesianIndex(0, size(img1, 2))
  map(m -> draw!(grid, LineSegment(m[1], m[2] + offset)), matches)
  grid
end


function matchAndMinimize(img1::AbstractMatrix{<:Gray}, img2::AbstractMatrix{<:Gray})
  keypoints_1 = Keypoints(fastcorners(img1, 15, 0.3))
  keypoints_2 = Keypoints(fastcorners(img2, 15, 0.3))

  brief_params = BRIEF(size = 256, window = 20, seed = 123)

  desc_1, ret_kps_1 = create_descriptor(img1, keypoints_1, brief_params);
  desc_2, ret_kps_2 = create_descriptor(img2, keypoints_2, brief_params);

  matches = match_keypoints(ret_kps_1, ret_kps_2, desc_1, desc_2, 0.1)

  pts1 = (x->x[1]).(matches) .|> x-> [x[1];x[2]]
  _pts2 = (x->x[2]).(matches)

  aTb = [1. 0 0; 0 1 0; 0 0 1]
  pts2 = transformIndexes(_pts2, aTb)

  res = Optim.optimize((xy)->cost_tr(xy, matches), [0.0;0.0])
  res.minimizer, matches
end


function pushMatchWindowMedian!(iCB::AbstractVector{<:AbstractMatrix{<:Gray}}, 
                                trBuffer::AbstractVector{<:AbstractVector},
                                newImg::AbstractMatrix{<:Gray} )
  #
  for bimg in iCB
    trans, matches = matchAndMinimize(newImg, bimg);
    push!(trBuffer, trans)
  end

  Ti = median((x->x[1]).(trBuffer))
  Tj = median((x->x[2]).(trBuffer))

  return [Ti; Tj]
end