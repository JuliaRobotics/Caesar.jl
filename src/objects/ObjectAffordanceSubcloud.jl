
#  Experimental
# export ObjectAffordanceSubcloud
# export makePointCloudObjectAffordance, generateObjectAffordanceFromWorld!
# export assembleObjectCache

# factor for mechanizing object affordances
Base.@kwdef struct _ObjAffSubcCache
  """ list of object subclouds in individual pose reference frames """
  p_SCs::Vector{<:_PCL.PointCloud}
  """ pose to object transforms for each indivudal pose variable """
  o_Ts_p::Vector{<:ArrayPartition}
  """ any object priors connected to the landmark variable """
  objPrior::Vector{Symbol} = Symbol[]
  # TODO add logic or necessary registers for the common object reference frame
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
  ohat_Ts_p::Vector{typeof(ArrayPartition(SA[0.;0.;0.],SMatrix{3,3}(diagm([1;1;1.]))))} = Vector{typeof(ArrayPartition(SA[0.;0.;0.],SMatrix{3,3}(diagm([1;1;1.]))))}()
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
#   elseif f == :ohat_T_p
#     # TODO, slow implementation which is inverting the homograph matrix twice.  Make better.
#     ArrayPartition(oas.p_BBo.position, oas.p_BBo.rotation)
#   else
#     error("ObjectAffordanceSubcloud does not have field $f")
#   end
# end

# ObjectAffordanceSubcloud(
#   p_BBo::_PCL.AbstractBoundingBox, 
#   ohat_T_p::ArrayPartition, 
#   p_PCloo_blobId::UUID
# ) = ObjectAffordanceSubcloud(p_BBo, ArrayPartition(SA[ohat_T_p.x[1]...],SMatrix{size(ohat_T_p.x[2])...}(ohat_T_p.x[2])), p_PCloo_blobId)

getManifold(oas::ObjectAffordanceSubcloud) = PowerManifold(getManifold(Pose3Pose3), NestedReplacingPowerRepresentation(), length(oas.ohat_Ts_p))

# function _findOASPriors()
#   # get all pose variables connected to this object
#   loovlb = getLabel(fvars[1])
#   olbl = getLabel(fvars[2])
#   # will not yet include this new factor.
#   aflb = ls(dfg,olbl)
#   neis = ls.(dfg,aflb)
#   avlbs = 0 < length(neis) ? union(neis...) : neis
#   avlbs = setdiff(avlbs, [getLabel(fvars[2]);])
#   # should make sure loovlb removed, since `IIF.rebuildFactorMetadata!` usage affects the lie loo caching steps
#   lievlbs = setdiff(avlbs, [loovlb;])
#   fc_lie_lbls = Symbol[]
  
#   # define the LIE elements
#   for liev in lievlbs
#     neifl = intersect(aflb,ls(dfg,liev))
#     # sibling factors
#     neifc = getFactorType.(dfg,neifl)
#     # must be OAS factor
#     filter!(f->f isa ObjectAffordanceSubcloud, neifc)
#     # filtering from all factors, skip this variable if not part of this object affordance 
#     0 === length(neifl) ? continue : nothing
#     nfclb = neifl[1]
#     nfc = neifc[1]
#     # collect factor labels
#     push!(fc_lie_lbls, nfclb)

#     # collect list of all subclouds
#     # p_SC = _PCL.getDataSubcloudLocal(dfg, liev, fct.p_BBo, r"PCLPointCloud2"; checkhash=false)
#     # push!(p_SClie, p_SC)
#     # # collect list of all pose to object transforms
#     # # FIXME, should start with current best guess provided by user
#     # push!(o_Tlie_p, nfc.ohat_T_p) # e0
#   end

# end


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
  APT = typeof(e0)
  o_Ts_p = APT[]

  cache = _ObjAffSubcCache(;
    p_SCs,
    o_Ts_p
  )
  
  # NOTE, obj variable first, pose variables are [2:end]
  for (i,vl) in enumerate(getLabel.(fvars)[2:end])
    p_PC = _PCL.getDataPointCloud(dfg, vl, fct.p_PC_blobIds[i]; checkhash=false) |> _PCL.PointCloud
    p_SC = _PCL.getSubcloud(p_PC, fct.p_BBos[i])

    ohat_T_p_ = fct.ohat_Ts_p[i]
    ohat_T_p = ArrayPartition(MVector(ohat_T_p_.x[1]...), MMatrix{3,3}(ohat_T_p_.x[2]))

    push!(cache.p_SCs, p_SC)
    push!(cache.o_Ts_p, ohat_T_p)
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
  if 0 < length(cache.o_Ts_p)
    oo_Ts_p = _PCL.alignPointCloudsLOOIters!(
      cache.o_Ts_p, 
      cache.p_SCs, 
      true, 3
    )
  end
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
  ohat_SCobj = _PCL.mergePointCloudsWithTransforms(cache.o_Ts_p, cache.p_SCs,)
  
  ohat_SCobj
end

function IncrementalInference.getSample(
  cf::CalcFactor{S},
) where {S <: ObjectAffordanceSubcloud}
  #
  M = getManifold(cf.factor).manifold
  e0 = ArrayPartition(SVector(1,1,1.),SMatrix{3,3}(diagm(ones(3))))
  len = length(cf.cache.o_Ts_p)
  
  Xs = Vector{typeof(e0)}(undef, len)
  
  # TODO, incorporate PointCloudPriors
  for i in 1:len
    # TBD consider adding a perturbation before alignment
    oo_Tloo_p, o_PClie, o_PCloo = _PCL.alignPointCloudLOO!(
      cf.cache.o_Ts_p,
      cf.cache.p_SCs,
      i;
      updateTloo=false
    )
    # TODO confirm expansion around e0, since PosePose factors expand around `q`
    X = log(M, e0, oo_Tloo_p)
    # make sure these are static arrays
    Xs[i] = ArrayPartition(SA[X.x[1]...], SMatrix{3,3}(X.x[2]))
  end
  # FIXME, need better stochastic calculation and representative covariance result in strong unimodal cases -- see this `getMeasurementParametric``
  # return the transform from pose to object as manifold tangent element
  
  return Xs
end


# FIXME, should be tangent vector not coordinates -- likely part of ManOpt upgrade
function (cf::CalcFactor{<:ObjectAffordanceSubcloud})(X, obj, qs...)
  # copied from Pose3Pose3
  PM = getManifold(cf.factor)
  M = PM.manifold # getManifold(Pose3)
  e0 = identity_element(M, obj)

  # NOTE allocalte for vee! see Manifolds #412, fix for AD
  q_Cqi = Vector{eltype(obj.x[1])}(undef, 6*length(qs))
  # q_Cqi = zeros(6*length(qs))
  for (i,qi) in enumerate(qs)
    # work in the group
    q̂i = Manifolds.compose(M, obj, exp(M, e0, X[i])) 
    idx = 6*(i-1)
    q_Xqi = log(M, qi, q̂i)
    # vee!(M, q_Cqi_, qi, q_Xqi)
    q_Ci = vee(M, qi, q_Xqi)
    q_Cqi[idx+1:idx+6] = q_Ci
  end
  # coordinates of all transforms to obj from pose observations
  return q_Cqi
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
  len = length(cache.o_Ts_p)
  μ = zeros(len*D_)
  # stack all alignments between object and each of the poses to this factor
  iΣ = zeros(len*D_,len*D_)
  iΣ_ = diagm([0.2*ones(3); 10*ones(3)])
  for (i,o_T_p) in enumerate(cache.o_Ts_p)
    # FIXME, not happy with inv on o_T_p -- OAS factor should read from pose to object???
    μ_ = vee(M, e0, log(M, e0, o_T_p )) #inv(M, o_T_p)))
    idx = D_*(i-1)+1
    μ[idx:(idx+D_-1)] = μ_
    iΣ[idx:(idx+D_-1),idx:(idx+D_-1)] = iΣ_
  end
  # basically a sort of ProductManifold, but not sure if this interfaces with Manifolds corretly
  μ, iΣ
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

  o_Ts_p = if align == :fine
    @warn("Using preamble fine alignment from $flb")
    cache.o_Ts_p
  else
    getFactorType(fc).ohat_Ts_p
  end

  ohat_SCobj = _PCL.mergePointCloudsWithTransforms(o_Ts_p, cache.p_SCs,)

  # M = getManifold(Pose3)
  # # the final output object point cloud object (in object frame)
  # ohat_SCobj = _PCL.PointCloud()

  # for flb in ls(dfg, ovl)
  #   # work from user and factor cache data
  #   fc = getFactor(dfg, flb)
  #   # get a factor subcloud of the object in the pose frame
  #   p_SC = IIF._getCCW(fc).dummyCache.p_SCloo[]
  #   # get the transform from pose to object frame
  #   o_T_p = if align == :fine
  #     @warn("Using preamble fine alignment from $flb")
  #     IIF._getCCW(fc).dummyCache.o_Tloo_p[]
  #   else
  #     getFactorType(dfg, flb).ohat_T_p
  #   end
  #   # convert the pose keyframe point cloud to 
  #   ohat_SC = _PCL.apply(M, o_T_p, p_SC)
  #   cat(
  #     ohat_SCobj,
  #     ohat_SC;
  #     reuse = true
  #   )
  # end

  return ohat_SCobj
end


function generateObjectAffordanceFromWorld!(
  dfg::AbstractDFG,
  olb::Symbol,
  vlbs::AbstractVector{<:Symbol},
  w_BBobj::_PCL.AbstractBoundingBox;
  solveKey::Symbol = :default,
  pcBlobLabel = r"PCLPointCloud2"
)
  M = SpecialEuclidean(3) # getManifold(Pose3)
  # add the object variable
  addVariable!(dfg, olb, Pose3)
  # add the object affordance subcloud factors
  oas = ObjectAffordanceSubcloud()

  for vlb in vlbs
    # make sure PPE is set on this solveKey
    setPPE!(dfg, vlb, solveKey)
    p_BBo, p_T_ohat = _PCL.transformFromWorldToLocal(dfg, vlb, w_BBobj; solveKey) # ohat_T_p
    ohat_T_p_ = inv(M, p_T_ohat)
    ohat_T_p = ArrayPartition(SA[ohat_T_p_.x[1]...], SMatrix{3,3}(ohat_T_p_.x[2]))
    p_PC_blobId = getDataEntry(dfg, vlb, pcBlobLabel).id
    push!(oas.p_BBos, p_BBo)
    push!(oas.ohat_Ts_p, ohat_T_p)
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