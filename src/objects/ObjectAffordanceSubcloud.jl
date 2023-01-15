
#  Experimental
# export ObjectAffordanceSubcloud
# export makePointCloudObjectAffordance, generateObjectAffordanceFromWorld!
# export assembleObjectCache

# factor for mechanizing object affordances
Base.@kwdef struct _ObjAffSubcCache
  """ list of object subclouds in individual pose reference frames """
  p_SCs::Vector{<:_PCL.PointCloud}
  """ pose to object transforms for each indivudal pose variable """
  lhat_Ts_p::Vector{<:ArrayPartition}
  """ individual transforms back from each subcloud reference to a common object reference frame """
  o_Ts_li::Vector{<:ArrayPartition}
  """ any object priors connected to the landmark variable """
  objPrior::Vector{Symbol} = Symbol[]
  # TODO add registers for ObjectAffordancePrior
end
# """ extracted subclouds from connected LIE factors """
# p_SClie::Vector{<:_PCL.PointCloud}
# """ list of transforms of pose in object (or pose to object) from connected factors """
# o_Tlie_p::Vector{<:ArrayPartition}



"""
    $TYPEDEF

Objects appear in lidar scans (bigger or smaller than the vehicle).  This factor masks a subcloud from the
variable point cloud and then tries to use that subcloud as a navigation affordance.  

Valid objects examples:
- The floor, wall, corner, window, etc
- A potplant standing in the room.
- An entire tree standing outside the room.
- The room itself around the vehicle.

Notes
- MAJOR QUIRK, given status at Caesar v0.13, adding OAS factors requires sibling 
  factors to `rebuildFactorMetadata!(..;_blockRecursionGradients=true)` once all factors are present in the graph.
  - because, `preambleCache` for `::OAS` does pre-emptive calculations based on current sibling factors.
- Bounding box definition is only local to this variable's lidar scan, therefore not 
  forcing the user to define any world frame assumptions before the map is built.


DevNotes:
- Which type of bounding boxes to use, hollow cube, ellipsoid, polytope, etc.
- Generalize for use with both Pose2 and Pose3 cases.
- TODO, in what reference frame is the object maintained?  The first variable?  What about LOO on pose1?
- TODO, MAKER_asfwe, allow more than one OAS solution to work in parallel on same object variable.
- FIXME, separate object frame can have LOO alignment wind up inside graph -- a wind-up release mechanism is needed
  whereby pose to refined object frame is consistent over all factors to object affordance variable.
"""
Base.@kwdef struct ObjectAffordanceSubcloud <: AbstractManifoldMinimize
  """ subcloud is selected by this mask from the variable's point cloud """
  p_BBos::Vector{<:_PCL.OrientedBoundingBox} = Vector{_PCL.OrientedBoundingBox}() 
  # _PCL.OrientedBoundingBox( SA[0,0,0.], SA[1,1,1.], SA[0,0,0.], SMatrix{3,3}(diagm([1;1;1.])) )
  """
  pose to object offset (or pose in object frame) to where the center of the object's bounding box. 
  I.e. the user provided initial guess of relative transform to go from pose to object frame.
  Note, the type here is restricted to StaticArrays only, to ensure best alignments are stored in the cache. """
  lhat_Ts_p::Vector{typeof(ArrayPartition(SA[0.;0.;0.],SMatrix{3,3}(diagm([1;1;1.]))))} = Vector{typeof(ArrayPartition(SA[0.;0.;0.],SMatrix{3,3}(diagm([1;1;1.]))))}()
  """
  standard entry blob label where to find the point clouds -- 
  forced to use getData since factor cache needs to update when other factors are 
  added to the same object variable later by the user.  See [`IIF.rebuildFactorMetadata!`](@ref).
  FIXME: there is a hack, should not be using Serialization.deserialize on a PCLPointCloud2, see Caesar.jl#921 """
  p_PC_blobIds::Vector{UUID} = Vector{UUID}()
end

# function Base.getproperty(oas::ObjectAffordanceSubcloud, f::Symbol)
#   if f == :p_BBo || f == :p_PCloo_blobId
#     getfield(oas, f)
#   elseif f == :bb_T_p
#     # TODO, slow implementation which is inverting the homograph matrix twice.  Make better.
#     ArrayPartition(oas.p_BBo.position, oas.p_BBo.rotation)
#   else
#     error("ObjectAffordanceSubcloud does not have field $f")
#   end
# end

# ObjectAffordanceSubcloud(
#   p_BBo::_PCL.AbstractBoundingBox, 
#   bb_T_p::ArrayPartition, 
#   p_PCloo_blobId::UUID
# ) = ObjectAffordanceSubcloud(p_BBo, ArrayPartition(SA[bb_T_p.x[1]...],SMatrix{size(bb_T_p.x[2])...}(bb_T_p.x[2])), p_PCloo_blobId)

getManifold(
  oas::ObjectAffordanceSubcloud
) = PowerManifold(
  getManifold(Pose3Pose3), 
  NestedReplacingPowerRepresentation(), 
  length(oas.lhat_Ts_p)
) # NOTE include length of priors which should also be used in measurement updates

struct ObjectModelPrior
  pc::_PCL.PointCloud
end

function _findObjPriors(dfg::AbstractDFG, fvars::AbstractVector{<:DFGVariable})
  objlb = getLabel(fvars[1])
  aflbs = ls(dfg,objlb)
  
  objpriors = Symbol[]
  
  for flb in aflbs
    neifc = getFactorType(dfg,flb)
    # must be OAS factor
    if neifc isa MetaPrior{<:ObjectModelPrior}
      push!(objpriors, flb)
    end
  end
  
  @assert length(objpriors) < 2 "Only one MetaPrior can be added to the object variable."
  if length(objpriors) == 1
    return objpriors, _PCL.getDataPointCloud(dfg, objpriors[1], r"PointCloudLAS"; checkhash=false)
  else
    return objpriors, nothing
  end
end


function _defaultOASCache(
  dfg::AbstractDFG, 
  fvars::AbstractVector{<:DFGVariable}, 
  fct::ObjectAffordanceSubcloud
)
  
  # use concrete types
  _PCT(::PC) where {PC <: _PCL.PointCloud} = PC
  PCT = _PCT(_PCL.PointCloud())
  p_SCs = PCT[]
  
  # make static to ensure future updates replace (not all LIEs reuse the same) memory
  e0 = ArrayPartition( MVector(0.,0.,0.), MMatrix{3,3}(diagm(ones(3))) )
  # e0_ = ArrayPartition( SVector(0.,0.,0.), SMatrix{3,3}(diagm(ones(3))) )
  APT = typeof(e0)
  lhat_Ts_p = Vector{APT}()
  o_Ts_li = Vector{APT}()

  # pre-emptively search for any meta object priors
  objlbs, o_PC = _findObjPriors(dfg, fvars)

  # start populating the data containers with necessary point clouds and transforms
  cache = _ObjAffSubcCache(;
    p_SCs,
    lhat_Ts_p,
    o_Ts_li
  )
  
  # NOTE, obj variable first, pose variables are [2:end]
  for (i,vl) in enumerate(getLabel.(fvars)[2:end])
    p_PC = _PCL.getDataPointCloud(dfg, vl, fct.p_PC_blobIds[i]; checkhash=false) |> _PCL.PointCloud
    p_SC = _PCL.getSubcloud(p_PC, fct.p_BBos[i])

    bb_T_p_ = fct.lhat_Ts_p[i]
    bb_T_p = ArrayPartition(
      MVector(bb_T_p_.x[1]...), 
      MMatrix{size(bb_T_p_.x[2])...}(bb_T_p_.x[2])
    )

    push!(cache.p_SCs, p_SC)
    push!(cache.lhat_Ts_p, bb_T_p)
  end
  
  # add any priors to back of list
  if 0 < length(objlbs)
    # NOTE assume just one model prior allowed (if present)
    push!(cache.p_SCs, o_PC)
    push!(cache.lhat_Ts_p, e0)
    push!(cache.objPrior, objlbs[1])
  end
  
  return cache
end


## TODO maybe overload addFactor! for ::ObjectAffordanceSubcloud so that when new factors are added 
# to the same object, each of the preambleCache functions can rerun, and thereby update with new loo 
# info, alternatively this update needs to be detect in the getSample functions which may not have 
# access to all the heavy lift data wrangling functions.  Solution is to readd Factors on loo list.

# NOTE Use this with rebuildFactorMetadata! to recompute cache on existing OAS 
#  factors when a new OAS factor is added to an object variable
# TODO, confirm fvars[2] works under multihypo use
function IncrementalInference.preambleCache(
  dfg::AbstractDFG, 
  fvars::AbstractVector{<:DFGVariable}, 
  fct::ObjectAffordanceSubcloud
)
  cache = _defaultOASCache(dfg, fvars, fct)
  # finalize object point clouds for cache
  # align if there if there is at least one LIE transform and cloud available.
  if 1 < length(cache.lhat_Ts_p)
    # manually set iterLength to skip possible priors, length(fvars) minus the object variable
    _PCL.alignPointCloudsLOOIters!(
      cache.lhat_Ts_p, 
      cache.p_SCs, 
      true, 3;
      iterLength=length(cache.lhat_Ts_p)-length(cache.objPrior) # skip priors
    )
  end

  # Assume fully aligned subclouds, and generate a new common object reference frame
  l_PC, o_Ts_li = _PCL.mergePointCloudsWithTransforms(cache.lhat_Ts_p, cache.p_SCs)
  # Chain of frames: from world to pose to localBoundingBox to relativeAlignment to object
  resize!(cache.o_Ts_li, length(o_Ts_li))
  for (i,oTl) in enumerate(o_Ts_li)
    cache.o_Ts_li[i] = oTl
  end

  # l_T_o = _PCL.calcAxes3D(l_PC)
  # M = getManifold(fvars[1])
  # # find all bb_T_o
  # compose(M, cache.lhat_Ts_p[i], ArrayPartition(fct.p_BBos[i].position, fct.p_BBos[i].orientation) )
  
  # update the cached memory pointers  
  return cache
end

"""
    $SIGNATURES

Debug tool to check values in the cache object make sense.  
Use before and after [`alignPointCloudsLOOIters!`](@ref).
"""
function assembleObjectCache(
  dfg::AbstractDFG, 
  flb::Symbol
)
  cache = IIF._getCCW(dfg, flb).dummyCache
  l_SCobj, o_Ts_li = _PCL.mergePointCloudsWithTransforms(cache.lhat_Ts_p, cache.p_SCs,)
  
  l_SCobj
end

function IncrementalInference.getSample(
  cf::CalcFactor{S},
) where {S <: ObjectAffordanceSubcloud}
  #
  M = getManifold(cf.factor).manifold
  e0 = ArrayPartition(SVector(1,1,1.),SMatrix{3,3}(1,0,0,0,1,0,0,0,1.))
  # LOO iterate over both relatives and priors
  iterLength = length(cf.cache.lhat_Ts_p) # - length(cf.cache.objPrior)
  
  # buffer to store LOO updated
  lhat_TTloo_p = Vector{typeof(e0)}(undef, iterLength)
    # # TODO generate a common "object frame" -- not the collection of all local bounding box references.
    # T_o = if 0 < length(cf.cache.objPrior)
    #   # use prior reference as object frame reference
    # else
    # end

  # NOTE cache.lhat_Ts_p and .p_SCs include the model priors at end of lists
  for i in 1:iterLength
    # TBD consider adding a perturbation before alignment
    # updateTloo=false to allow future multithreading support
    lhat_Tloo_p, o_PClie, o_PCloo = _PCL.alignPointCloudLOO!(
      cf.cache.lhat_Ts_p,
      cf.cache.p_SCs,
      i;
      updateTloo=false
    )
    
    # TODO confirm expansion around e0, since PosePose factors expand around `q`
    # make sure these are static arrays
    lhat_TTloo_p[i] = ArrayPartition(SA[lhat_Tloo_p.x[1]...], SMatrix{3,3}(lhat_Tloo_p.x[2]))
  end
  
  # Assume fully aligned subclouds, and generate a new common object reference frame
  l_PC, o_Ts_li = _PCL.mergePointCloudsWithTransforms(lhat_TTloo_p, cf.cache.p_SCs)

  # TOWARDS stochastic calculation for a strong unimodal object frame -- see `getMeasurementParametric` for this factor
  w_XXop = similar(lhat_TTloo_p)
  for (i,lTp) in enumerate(lhat_TTloo_p) # cf.cache.o_Ts_li
    w_XXop[i] = log(M, e0, Manifolds.compose(M, o_Ts_li[i], lTp))
  end
  
  # return the transform from pose to object as manifold tangent element
  return w_XXop # lhat_Ts_p
end


IIF.getMeasurementParametric(oas::ObjectAffordanceSubcloud) = error("Special case on ObjectAffordanceSubcloud, use lower dispatch `getMeasurementParametric(::DFGFactor{CCW{<:ObjectAffordanceSubcloud}})` instead.")
function IIF.getMeasurementParametric(foas::DFGFactor{<:CommonConvWrapper{<:ObjectAffordanceSubcloud}})
  @warn "Only artificial inverse covariance available for `getMeasurementParametric(::DFGFactor{CCW{<:ObjectAffordanceSubcloud}})`" maxlog=3
  # TODO this only work for SpecialEuclidean(3), and half implemented for SpecialEuclidean(2)
  PM = getManifold(getFactorType(foas))
  M = PM.manifold
  D_ = manifold_dimension(M)
  e0 = ArrayPartition(SA[0;0;0.], SMatrix{3,3}([1 0 0; 0 1 0; 0 0 1.]))
  # accept the preambleCache alignment for parametric solves
  cache = IIF._getCCW(foas).dummyCache
  len = length(cache.lhat_Ts_p)
  w_Clps = zeros(len*D_)
  # stack all alignments between object and each of the poses to this factor
  w_iΣ = zeros(len*D_,len*D_)
  # inverse covariance matrix, 1/5 on position, 1/0.1 for rotation
  w_iΣ_ = diagm([0.2*ones(3); 10*ones(3)])  ## FIXME
  for (i,li_T_p) in enumerate(cache.lhat_Ts_p)
    # FIXME, should reference to a common object frame not the collection of bounding box references.
    # FIXME, not happy with inv on li_T_p -- OAS factor should read from pose to object???
    # w_Clps_ = vee(M, e0, log(M, e0, Manifolds.compose(M, cache.o_Ts_li[i], li_T_p) )) #inv(M, li_T_p)))
    w_Clp = vee(M, e0, log(M, e0, li_T_p )) #inv(M, li_T_p)))
    idx = D_*(i-1)+1
    w_Clps[idx:(idx+D_-1)] = w_Clp
    w_iΣ[idx:(idx+D_-1),idx:(idx+D_-1)] = w_iΣ_
  end
  # basically a sort of ProductManifold, but not sure if this interfaces with Manifolds corretly
  w_Clps, w_iΣ
end


# FIXME, should be tangent vector not coordinates -- likely part of ManOpt upgrade
function (cf::CalcFactor{<:ObjectAffordanceSubcloud})(w_Xlps, w_T_o, w_Ts_p...)
  # copied from Pose3Pose3
  PM = getManifold(cf.factor)
  M = PM.manifold # getManifold(Pose3)
  e0 = identity_element(M, w_T_o)

  # NOTE allocalte for vee! see Manifolds #412, fix for AD
  p_Cphats = Vector{eltype(w_T_o.x[1])}(undef, 6*length(w_Ts_p))
  # q_Cqi = zeros(6*length(w_Ts_p))
  for (i,w_T_p) in enumerate(w_Ts_p)
    # work in the group
    l_T_p = exp(M, e0, w_Xlps[i])
    w_T_phat = Manifolds.compose(M, w_T_o, l_T_p) # FIXME, single object frame, not local frame
    idx = 6*(i-1)
    # expanding phat around p seems stange?
    p_Xphat = log(M, w_T_p, w_T_phat)
    p_Cphat = vee(M, w_T_p, p_Xphat)
    p_Cphats[idx+1:idx+6] = p_Cphat
  end
  # coordinates of all transforms to obj from pose observations
  return p_Cphats
end

## =================================================================================
## Object Utils
## =================================================================================


"""
    $SIGNATURES

Put the object subclouds together in the object frame based on info stored in the factor graph 
around the object variable `ovl`.
"""
function makePointCloudObjectAffordance(
  dfg::AbstractDFG,
  flb::Symbol;
  align=:fine # or :coarse
)
    fc = getFactor(dfg, flb)
  cache = IIF._getCCW(fc).dummyCache

  lhat_Ts_p = if align == :fine
    @warn("Using preamble fine alignment from $flb")
    cache.lhat_Ts_p
  else
    getFactorType(fc).lhat_Ts_p
  end

  l_SCobj, o_Ts_li = _PCL.mergePointCloudsWithTransforms(lhat_Ts_p, cache.p_SCs)

  return l_SCobj
end


function generateObjectAffordanceFromWorld!(
  dfg::AbstractDFG,
  olb::Symbol,
  vlbs::AbstractVector{<:Symbol},
  w_BBobj::_PCL.AbstractBoundingBox;
  solveKey::Symbol = :default,
  pcBlobLabel = r"PCLPointCloud2",
  modelprior::Union{Nothing,<:_PCL.PointCloud}=nothing
)
  M = SpecialEuclidean(3) # getManifold(Pose3)
  # add the object variable
  addVariable!(dfg, olb, Pose3)

  # if a model prior exists, add it here
  if !isnothing(modelprior)
    mpr = MetaPrior(;data=ObjectModelPrior(modelprior))
    addFactor!(dfg, [olb;], mpr; tags=[:MODELPRIOR])
  end

  # add the object affordance subcloud factors
  oas = ObjectAffordanceSubcloud()

  for vlb in vlbs
    # make sure PPE is set on this solveKey
    setPPE!(dfg, vlb, solveKey)
    # lhat frame is some local frame where object subclouds roughly fall together (could be host start from world frame)
    p_BBo, p_T_lhat = _PCL.transformFromWorldToLocal(dfg, vlb, w_BBobj; solveKey)
    lhat_T_p_ = inv(M, p_T_lhat)
    # make immutable data type
    lhat_T_p = ArrayPartition(SA[lhat_T_p_.x[1]...], SMatrix{3,3}(lhat_T_p_.x[2]))
    p_PC_blobId = getDataEntry(dfg, vlb, pcBlobLabel).id
    push!(oas.p_BBos, p_BBo)
    push!(oas.lhat_Ts_p, lhat_T_p)
    push!(oas.p_PC_blobIds, p_PC_blobId)
  end
  
  addFactor!(dfg, [olb; vlbs], oas)
  
  oas
end


function reducePoseGraphOnObjectAffordances!(
  dest::AbstractDFG,
  src::AbstractDFG,
)
  #
  vlbs = ls(src) |> sortDFG
  flbs = lsf(src)

  # remove OAS types
  oasls = filter(f->getFactorType(src,f) isa Caesar.ObjectAffordanceSubcloud, flbs)

  # copy all but OAS factors
  copyGraph!(dest, src, vlbs, setdiff(flbs,oasls))

  # re-add OAS as pairwise PosePose
  for oasl in oasls
    fc = getFactor(src, oasl)
    D_ = getManifold(getFactorType(fc)).manifold |> manifold_dimension
    μ, iΣ = getMeasurementParametric(fc)
    ovlbs = getVariableOrder(fc)
    for (i,vl) in enumerate(ovlbs[2:end])
      idx = D_*(i-1)
      mn = μ[idx+1:idx+D_]
      sm = iΣ[idx+1:idx+D_,idx+1:idx+D_] |> inv
      addFactor!(dest, [ovlbs[1];vl], Pose3Pose3(MvNormal(mn, sm)); tags=[:OAS_POSEPOSE_REDUCTION])
    end
  end

  dest
end

function reducePoseGraphOnObjectAffordances(
  src::AbstractDFG;
  graphinit::Bool=false
)
  dest = initfg()
  getSolverParams(dest).graphinit = graphinit
  reducePoseGraphOnObjectAffordances!(dest, src)
end

##