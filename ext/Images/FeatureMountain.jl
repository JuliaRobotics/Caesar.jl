




function addFeatureTracks_Frame1_Q!(
  # mountain::FeatureMountain,
  featToMany,
  dfg::AbstractDFG, 
  vlb_q::Symbol;
  trackBlobKey = r"IMG_FEATURE_TRACKS_FWDBCK",
)
  #
  # if !haskey(mountain, 1)
  #   mountain[1] = FeatureTracks()
  # end
  eb = getData(dfg,vlb_q,trackBlobKey)
  if isnothing(eb)
    return featToMany
  end
  img_tracks = JSON3.read(String(eb[2]), ImageTracks)
  
  for (featidx, meas) in enumerate(img_tracks[0])
    # add image features that appear in one frame
    track_id = eb[1].originId # uuid4()
    pixel = (meas[1],meas[2])
    from = (vlb_q,featidx, pixel)
    to = (vlb_q,featidx, pixel)
      # # legcay
      # mountain[1][track_id] = FeaturesDict()
      # mountain[1][track_id][vlb_q] = (;from, to)

    # featToMany container
    fromkey = (from[1],from[2])
    if !haskey(featToMany, fromkey)
      featToMany[fromkey] = MANYTRACKS()
    end
    tokey = (to[1],to[2])
    flow = (from[1],0)
    flowedpix = pixel
    pixeltrack = (;via=to, track=(track_id,flow,flowedpix))
    featToMany[fromkey][tokey] = pixeltrack
  end
  
  return featToMany
  # return mountain
end


function addFeatureTracks_Frame2_PfwdQ!(
  # mountain::FeatureMountain,
  featToMany,
  dfg::AbstractDFG,
  vlb_pq,
  mask = nothing;
  trackBlobKey = r"IMG_FEATURE_TRACKS_FWDBCK_",
  imgBlobKey = r"cam"
)

  # if !haskey(mountain, 2)
  #   mountain[2] = FeatureTracks()
  # end

  vlb_p, vlb_q = vlb_pq[1], vlb_pq[2]

  ie_ib = getData(dfg, vlb_q, imgBlobKey)
  img_q = unpackBlob(MIME(ie_ib[1].mimeType), ie_ib[2])
  maskl = isnothing(mask) ? zeros(Gray{N0f8}, size(img_q)...) : mask
  mimg_q = Caesar.applyMaskImage(img_q, maskl .< 0.5)

  eb_q = getData(dfg, vlb_q, trackBlobKey)
  eb_p = getData(dfg, vlb_p, trackBlobKey)
  if isnothing(eb_p) || isnothing(eb_q)
    return featToMany
  end
  tracks_q = JSON3.read(String(eb_q[2]), ImageTracks)
  tracks_p = JSON3.read(String(eb_p[2]), ImageTracks)

  # KLTFWD w/ DESC
  tracks_pP1_q0, idx_pP1_q0 = curateFeatureTracks(tracks_p[1], tracks_q[0], img_q, mimg_q)

  for (i_pP1_q0, trk_pP1_q0) in zip(idx_pP1_q0, tracks_pP1_q0)
    # add image features that appear in one frame
    track_id = eb_p[1].originId # uuid4()
    pixel_p = tuple(tracks_p[0][i_pP1_q0[1]]...)
    pixel_q = tuple(trk_pP1_q0[2]...)
    from = (vlb_p, i_pP1_q0[1], pixel_p)
    to = (vlb_q, i_pP1_q0[2], pixel_q)
      # # legacy
      # mountain[2][track_id] = FeaturesDict()
      # mountain[2][track_id][vlb_p] = (;from, to)
      # mountain[2][track_id][vlb_q] = (;from, to)

    # featToMany container
    fromkey = (from[1],from[2])
    if !haskey(featToMany, fromkey)
      featToMany[fromkey] = MANYTRACKS()
    end
    tokey = (to[1],to[2])
    flow = (from[1],+1)
    flowedpix = tuple(tracks_p[1][i_pP1_q0[1]]...)
    pixeltrack = (;via=to, track=(track_id,flow,flowedpix))
    featToMany[fromkey][tokey] = pixeltrack
  end

  return featToMany
  # return mountain
end


function addFeatureTracks_Frame2_QbckR!(
  # mountain::FeatureMountain,
  featToMany,
  dfg::AbstractDFG,
  vlb_qr,
  mask = nothing;
  trackBlobKey = r"IMG_FEATURE_TRACKS_FWDBCK_",
  imgBlobKey = r"cam"
)

  # if !haskey(mountain, 2)
  #   mountain[2] = FeatureTracks()
  # end

  vlb_q, vlb_r = vlb_qr[1], vlb_qr[2]

  ie_ib = getData(dfg, vlb_q, imgBlobKey)
  img_q = unpackBlob(MIME(ie_ib[1].mimeType), ie_ib[2])
  maskl = isnothing(mask) ? zeros(Gray{N0f8}, size(img_q)...) : mask
  mimg_q = Caesar.applyMaskImage(img_q, maskl .< 0.5)

  eb_q = getData(dfg, vlb_q, trackBlobKey)
  eb_r = getData(dfg, vlb_r, trackBlobKey)
  if isnothing(eb_r) || isnothing(eb_q)
    return featToMany
  end
  tracks_q = JSON3.read(String(eb_q[2]), ImageTracks)
  tracks_r = JSON3.read(String(eb_r[2]), ImageTracks)

  # KLTBCK w/ DESC
  # tracks_q0_rN1, idx_q0_rN1 = curateFeatureTracks(tracks_q[0], tracks_r[-1], img_q, mimg_q)
  tracks_rN1_q0, idx_rN1_q0 = curateFeatureTracks(tracks_r[-1], tracks_q[0], img_q, mimg_q)

  # # KLTFWD w/ DESC
  # tracks_pP1_q0, idx_pP1_q0 = curateFeatureTracks(tracks_p[1], tracks_q[0], img_q, mimg_q)

  for (i_rN1_q0, trk_rN1_q0) in zip(idx_rN1_q0, tracks_rN1_q0)
    # add image features that appear in one frame
    track_id = eb_r[1].originId # uuid4()
    pixel_r = tuple(tracks_r[0][i_rN1_q0[1]]...)
    pixel_q = tuple(trk_rN1_q0[2]...)
    from = (vlb_r, i_rN1_q0[1], pixel_r)
    to = (vlb_q, i_rN1_q0[2], pixel_q)
      # # legacy
      # mountain[2][track_id] = FeaturesDict()
      # mountain[2][track_id][vlb_r] = (; from, to)
      # mountain[2][track_id][vlb_q] = (; from, to)
    
    # featToMany container
    fromkey = (from[1],from[2])
    if !haskey(featToMany, fromkey)
      featToMany[fromkey] = MANYTRACKS()
    end
    tokey = (to[1],to[2])
    flow = (from[1],-1)
    flowedpix = tuple(tracks_r[-1][i_rN1_q0[1]]...)
    pixeltrack = (;via=to, track=(track_id,flow,flowedpix))
    featToMany[fromkey][tokey] = pixeltrack
  end

  return featToMany
  # return mountain
end

# # require both forward and backward tracks to coincide
# function addFeatureTracks_Frame2!(
#   mountain::FeatureMountain,
#   dfg::AbstractDFG,
#   vlb_qr,
#   trackBlobKey = r"IMG_FEATURE_TRACKS_FWDBCK_",
# )
  
#   _featM = FeatureMountain()
  
#   # FWD pairs
#   addFeatureTracks_Frame2_PfwdQ!(_featM, dfg, vlb_pq)
#   # BCK pairs
#   addFeatureTracks_Frame2_QbckR!(_featM, dfg, vlb_pq)

#   # now get tracking from previous through current to next
#   intsc_q0 = intersect((s->s[2]).(idx_pP1_q0), (s->s[1]).(idx_q0_rN1))

#   tracks_pP1_rN1 = Pair{Int,Int}[]
#   for com_idx in intsc_q0
#     idx_tr_pP1_q0 = idx_pP1_q0[findfirst(==(com_idx),(s->s[2]).(idx_pP1_q0))][1]
#     idx_tr_q0_rN1 = idx_q0_rN1[findfirst(==(com_idx),(s->s[1]).(idx_q0_rN1))][2]
#     push!(tracks_pP1_rN1, idx_tr_pP1_q0 => idx_tr_q0_rN1)
#   end


#   tracks_pP1_rN1

# end




## ==================================================================
## Build feature mountain
## ==================================================================



## process based feature construction
# 1. add features seen in one frame
# 2a. add features tracked 2-frame feature tracks (i.e. between two seccessive images) 
# 2b. remove duplicate 2-frame features (may have originated from different images)
# 3a. consolidate into any 3-frame image features with new feature uuid
# 3b. remove duplicate 3-frame features
# 4a. consolidate into any 4-frame image features with new feature uuid
# 4b. remove duplicate 4-frame features
# ...


"""
    $SIGNATURES

Add features per image (frame1) and pairs (frame2).

```julia
# FWD pairs and BCK pairs
N = 500
pairs = zip(
  [Symbol("xl\$i") for i in (500-1):(500-1+N)], 
  [Symbol("xl\$i") for i in 500:(500+N)]
)

featM = addFeatureTracks(fg, pairs)
```
"""
function addFeatureTracks(
  dfg::AbstractDFG,
  pairs,
  mask = nothing;
  trackBlobKey = r"IMG_FEATURE_TRACKS_FWDBCK_",
  imgBlobKey = r"cam"
)
  #
  featToMany_ = Dict{Tuple{Symbol,Int},MANYTRACKS}()

  lastlbl = :null
  @showprogress "feature pairs" for pair in pairs
    addFeatureTracks_Frame1_Q!(featToMany_, dfg, pair[1]; trackBlobKey)
    addFeatureTracks_Frame2_PfwdQ!(featToMany_, dfg, pair, mask; trackBlobKey, imgBlobKey)
    addFeatureTracks_Frame2_QbckR!(featToMany_, dfg, pair, mask; trackBlobKey, imgBlobKey)  
    lastlbl = pair[2]
  end
  addFeatureTracks_Frame1_Q!(featToMany_, dfg, lastlbl; trackBlobKey)

  return featToMany_
end
# # add all features seen in each frame with unique uuid
# addFeatureTracks_Frame1_Q!(featToMany_, fg, :xl499)
# addFeatureTracks_Frame1_Q!(featToMany_, fg, :xl500)
# addFeatureTracks_Frame1_Q!(featToMany_, fg, :xl501)
# addFeatureTracks_Frame1_Q!(featToMany_, fg, :xl502)
# # addFeatureTracks_Frame1_Q!.(Ref(featM), fg, [Symbol("xl$(500+i)") for i in -1:1])

# # FWD pairs and BCK pairs
# pair = (:xl499,:xl500)
# addFeatureTracks_Frame2_PfwdQ!(featToMany_, fg, pair)
# addFeatureTracks_Frame2_QbckR!(featToMany_, fg, pair)

# pair = (:xl500,:xl501)
# addFeatureTracks_Frame2_PfwdQ!(featToMany_, fg, pair)
# addFeatureTracks_Frame2_QbckR!(featToMany_, fg, pair)

# pair = (:xl501,:xl502)
# addFeatureTracks_Frame2_PfwdQ!(featToMany_, fg, pair)
# addFeatureTracks_Frame2_QbckR!(featToMany_, fg, pair)



function consolidateFeatureTracks!(
  featToMany_::Dict{Tuple{Symbol,Int},MANYTRACKS},
)
  ## find Frame3 options

  ssrck = sort(collect(keys(featToMany_)); by=s->s[1], lt=DFG.natural_lt)

  nchanges = 1
  while 0 < nchanges
    nchanges = 0
    # loop through all primary neighbor possiblities
    @showprogress "explore tracks" for srcfe in ssrck
      sval = featToMany_[srcfe]
      for (dstfe, dval) in sval # featToMany_[srcfe] # , (n2fe, n2val) in featToMany_[dstfe]
      
      # is there an already tracked primary neighbor
      # if srcfe != dstfe
        mergen2 = false
        # check if secondary neighbors exist
        for (n2fe, n2val) in featToMany_[dstfe]
          # check different feature than src or dst, and check not already in src list
          if srcfe != dstfe && n2fe != srcfe && !haskey(featToMany_[srcfe], n2fe)
            mergen2 = true
          end
        end
        if mergen2
          for (n2fe, n2val) in featToMany_[dstfe]
            if !haskey(featToMany_[srcfe], n2fe)
              featToMany_[srcfe][n2fe] = n2val # n2val_prime
              nchanges += 1
            end
          end
          # # remove dstfe
          # frame1 = featToMany_[dstfe][dstfe]
          # delete!(featToMany_, dstfe)
          # featToMany_[dstfe] = MANYTRACKS()
          # featToMany_[dstfe][srcfe] = sval[srcfe]
          # # keep self reference
          # featToMany_[dstfe][dstfe] = frame1
        end
        # add secondary neighbor to src list
        # n2val_prime = (; via = dval.via, track = n2val.track) #(n2val.track[1], n2val.track[2], n2val.track[3]))
        # remove the 
        # @info "adding secondary neighbor" srcfe n2fe
      end
    end
    @show nchanges
  end

  return featToMany_
end



function summarizeFeatureTracks!(
  featToMany_::Dict{Tuple{Symbol,Int},MANYTRACKS},
)
## summarize tracks to start label

  ssrck = sort(collect(keys(featToMany_)); by=s->s[1], lt=DFG.natural_lt)

  alltracks = Vector{Tuple{Symbol,Int}}()

  @showprogress "remove track duplication" for srcfe in ssrck
    if haskey(featToMany_, srcfe)
      sval = featToMany_[srcfe]
      for (dstfe, dval) in sval
        if srcfe != dstfe
          # identify new track
          if !(dstfe in alltracks)
            tmpfe = featToMany_[dstfe][dstfe]
            delete!(featToMany_, dstfe)
            # self ref
            featToMany_[dstfe] = MANYTRACKS()
            featToMany_[dstfe][dstfe] = tmpfe
            # track start ref
            union!(alltracks, [srcfe;])
            union!(alltracks, [dstfe;])
            featToMany_[dstfe][srcfe] = featToMany_[srcfe][srcfe]
          end
        end
      end
    end
  end

  return featToMany_
end



function buildFeatureMountain(
  dfg::AbstractDFG,
  vlbs::AbstractVector{Symbol},
  mask = nothing;
  trackBlobKey = r"IMG_FEATURE_TRACKS_FWDBCK_",
  imgBlobKey = r"cam"
)
  # adjacent variable labels with images
  pairs = zip(vlbs[1:end-1],vlbs[2:end])
  # generate feature mountain with frames 1 and 2 fwdbck
  featM = addFeatureTracks(dfg, pairs, mask; trackBlobKey, imgBlobKey)

  consolidateFeatureTracks!(featM)
  summarizeFeatureTracks!(featM)

  ## what should the final feature lookup look like after curation?
  return featM
end


## union features



function unionFeatureMountain(
  fMa::Dict{Tuple{Symbol,Int},MANYTRACKS}, 
  fMb::Dict{Tuple{Symbol,Int},MANYTRACKS},
)
  # start with everything from fMb
  rM = deepcopy(fMb)
  # then add everything from fMa
  for (ka,va) in fMa
    # @info ka
    # union if already exists
    if haskey(rM, ka)
      # @info "CHECK TYPES" typeof(fMa[ka]) typeof(rM[ka])
      for (kr,vr) in va
        if !haskey(rM[ka], kr)
          rM[ka][kr] = vr # union(fMa[ka], rM[ka])
        end
      end
    else
      rM[ka] = va
    end
  end
  return rM
end



function sortKeysMinSighting(
  featM::Dict{Tuple{Symbol,Int},<:Any};
  minSightings::Int = 1
)
  kM_mS = if minSightings == 1
    collect(keys(featM))
  else
    collect(keys(featM))[minSightings .< (keys(featM) .|> s->length(featM[s]))]
  end
  kM_mS_ = sort(kM_mS; by=s->s[2])
  return sort(kM_mS_; by=s->s[1], lt=DFG.natural_lt)
end




function buildFeatureMountainDistributed(
  dfg::AbstractDFG,
  vlbls::AbstractVector{Symbol},
  imgmask;
  STRIDE = 100
)
  WP = WorkerPool(collect(1:nprocs()))

  @show ntopr = ceil(Int,length(vlbls)/STRIDE)
  @show len = length(vlbls)

  chunks1 = Iterators.partition(vlbls, len ÷ ntopr) |> collect
  nosingles = findall(==(1), length.(chunks1))
  chunks1 = chunks1[setdiff(1:length(chunks1), nosingles)]
  tasks1 = map(chunks1) do vlbs1
    remotecall(Caesar.buildFeatureMountain, WP, dfg, vlbs1, imgmask)
  end
  chunks2 = Iterators.partition(vlbls[(1 + STRIDE÷2):end], len ÷ ntopr) |> collect
  nosingles = findall(==(1), length.(chunks2))
  chunks2 = chunks2[setdiff(1:length(chunks2), nosingles)]
  tasks2 = map(chunks2) do vlbs2
    remotecall(Caesar.buildFeatureMountain, WP, dfg, vlbs2, imgmask)
  end

  ## consolidate equivalent features

  # fetch all the results
  featM_1 = fetch.(tasks1)
  featM_2 = fetch.(tasks2)
  # featM_ = (hcat(featM_1, featM_2))'[:]
  # consolidate into common feature mountain container
  # start with first result as featM
  featM = deepcopy(featM_1[1])
  # union other tracks into featM
  for fM in featM_1[2:end]
    featM = Caesar.unionFeatureMountain(featM, fM)
  end
  for fM in featM_2
    featM = Caesar.unionFeatureMountain(featM, fM)
  end

  return featM


end




## Old dev code

# ## Curate 4oo4 KLTFWD-DESC-KLTBCK-DESC
# ## =======


# trackBlobKey = r"IMG_FEATURE_TRACKS_FWDBCK_"

# i=0
# vlb_q = Symbol("xl$(500+i)")

# ie_ib = getData(fg, vlb_q, r"cam01AEA9_")
# img_q = unpackBlob(MIME(ie_ib[1].mimeType), ie_ib[2])
# mimg_q = Caesar.applyMaskImage(img_q, maskl .< 0.5)

# eb = getData(fg, vlb_q, trackBlobKey)
# tracks_q = JSON3.read(String(eb[2]), ImageTracks)

# i=1
# vlb_p = Symbol("xl$(500-i)")
# eb = getData(fg, vlb_p, trackBlobKey)
# tracks_p = JSON3.read(String(eb[2]), ImageTracks)

# # KLTBCK
# vlb_r = Symbol("xl$(500+i)")
# eb = getData(fg, vlb_r, trackBlobKey)
# tracks_r = JSON3.read(String(eb[2]), ImageTracks)

# # KLTFWD w/ DESC
# tracks_pP1_q0, idx_pP1_q0 = curateFeatureTracks(tracks_p[1], tracks_q[0], img_q, mimg_q)
# # KLTBCK w/ DESC
# tracks_q0_rN1, idx_q0_rN1 = curateFeatureTracks(tracks_q[0], tracks_r[-1], img_q, mimg_q)
# tracks_rN1_q0, idx_rN1_q0 = curateFeatureTracks(tracks_r[-1], tracks_p[0], img_q, mimg_q)

# # now get tracking from previous through current to next
# intsc_q0 = intersect((s->s[2]).(idx_pP1_q0), (s->s[2]).(idx_rN1_q0))

# tracks_pP1_rN1 = Pair{Int,Int}[]
# for com_idx in intsc_q0
#   idx_tr_pP1_q0 = idx_pP1_q0[findfirst(==(com_idx),(s->s[2]).(idx_pP1_q0))][1]
#   # idx_tr_q0_rN1 = idx_q0_rN1[findfirst(==(com_idx),(s->s[1]).(idx_q0_rN1))][2]
#   idx_tr_rN1_q0 = idx_rN1_q0[findfirst(==(com_idx),(s->s[2]).(idx_rN1_q0))][1]
#   push!(tracks_pP1_rN1, idx_tr_pP1_q0 => idx_tr_rN1_q0)
# end


# tracks_pP1_rN1

# ## check

# [norm(tracks_p[1][pP1] - tracks_r[-1][rN1]) for (pP1,rN1) in tracks_pP1_rN1]



##