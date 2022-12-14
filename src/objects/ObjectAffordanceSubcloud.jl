
#  Experimental
# export ObjectAffordanceSubcloud
# export makePointCloudObjectAffordance, generateObjectAffordanceFromWorld!

# factor for mechanizing object affordances
Base.@kwdef struct _ObjAffSubcCache
  """ extracted LOO subcloud """
  p_SCloo::Base.RefValue{<:_PCL.PointCloud}
  """ LOO pose to object transform """
  o_Tloo_p::Base.RefValue{<:ArrayPartition}
  """ similar LIE factors connected to the same landmark variable """
  fc_lie_lbls::Vector{Symbol}
  """ extracted subclouds from connected LIE factors """
  p_SClie::Vector{<:_PCL.PointCloud}
  """ list of transforms of pose in object (or pose to object) from connected factors """
  o_Tlie_p::Vector{<:ArrayPartition}
end


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
Base.@kwdef struct ObjectAffordanceSubcloud{B} <: AbstractManifoldMinimize
  """ subcloud is selected by this mask from the variable's point cloud """
  p_BBo::B = _PCL.AxisAlignedBoundingBox( GeoB.Rect([GeoB.Point(0,0,0.),GeoB.Point(1,1,1.)]) )
  """
  pose to object offset (or pose in object frame) to where the center of the object's bounding box. 
  I.e. the user provided initial guess of relative transform to go from pose to object frame.
  Note, the type here is restricted to StaticArrays only, to ensure best alignments are stored in the cache. """
  ohat_T_p::typeof(ArrayPartition(SA[0.;0.;0.],SMatrix{3,3}(diagm([1;1;1.])))) = ArrayPartition(SA[0.;0.;0.],SMatrix{3,3}(diagm([1;1;1.])))
  """
  standard entry blob label where to find the point clouds -- 
  forced to use getData since factor cache needs to update when other factors are 
  added to the same object variable later by the user.  See [`IIF.rebuildFactorMetadata!`](@ref).
  FIXME: there is a hack, should not be using Serialization.deserialize on a PCLPointCloud2, see Caesar.jl#921 """
  p_PCloo_blobId::UUID
end

getManifold(::ObjectAffordanceSubcloud) = getManifold(Pose3Pose3)

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
  # get all pose variables connected to this object
  loovlb = getLabel(fvars[1])
  olbl = getLabel(fvars[2])
  # will not yet include this new factor.
  aflb = ls(dfg,olbl)
  neis = ls.(dfg,aflb)
  avlbs = 0 < length(neis) ? union(neis...) : neis
  avlbs = setdiff(avlbs, [getLabel(fvars[2]);])
  # should make sure loovlb removed, since `IIF.rebuildFactorMetadata!` usage affects the lie loo caching steps
  lievlbs = setdiff(avlbs, [loovlb;])
  fc_lie_lbls = Symbol[]
  
  # use concrete types
  _PCT(::PC) where {PC <: _PCL.PointCloud} = PC
  PCT = _PCT(_PCL.PointCloud())
  p_SClie = PCT[]
  
  # make static to ensure future updates replace (not all LIEs reuse the same) memory
  e0 = ArrayPartition(SA[0.;0.;0.],SMatrix{3,3}(diagm(ones(3))))
  APT = typeof(e0)
  o_Tlie_p = APT[]

  # define LOO element
  # NOTE, when updating caches and blobId error on x1, use `rebuildFactorMetadata!(..;_blockRecursionGradients=true)`
  p_PC = _PCL.getDataPointCloud(dfg, loovlb, fct.p_PCloo_blobId; checkhash=false) |> _PCL.PointCloud
  @show fct.p_BBo
  _p_SC = _PCL.getSubcloud(p_PC, fct.p_BBo)
  p_SCloo = Ref(_p_SC)
  o_Tloo_p = Ref(fct.ohat_T_p) # e0
  
  # define the LIE elements
  for (idx,liev) in enumerate(lievlbs)
    neifl = intersect(aflb,ls(dfg,liev))
    # sibling factors
    neifc = getFactorType.(dfg,neifl)
    # must be OAS factor
    filter!(f->f isa ObjectAffordanceSubcloud, neifc)
    # filtering from all factors, skip this variable if not part of this object affordance 
    0 === length(neifl) ? continue : nothing
    nfclb = neifl[1]
    nfc = neifc[1]
    # collect factor labels
    push!(fc_lie_lbls, nfclb)
    # find self (loo) number from factor list
    A, B = lsf(dfg, nfclb), getLabel.(fvars)
    0 === length(setdiff(A,B)) ? loo_idx = idx : nothing
    # collect list of all subclouds
    p_SC = _PCL.getDataSubcloudLocal(dfg, liev, fct.p_BBo, r"PCLPointCloud2"; checkhash=false)
    push!(p_SClie, p_SC)
    # collect list of all pose to object transforms
    # FIXME, should start with current best guess provided by user
    push!(o_Tlie_p, nfc.ohat_T_p) # e0
  end
  
  # finalize object point clouds for cache
  # align if there if there is at least one LIE transform and cloud available.
  if 0 < length(o_Tlie_p)
    # stack all the elements
    o_Ts_p = typeof(o_Tloo_p[])[o_Tloo_p[], o_Tlie_p...]
    p_SCs = typeof(p_SCloo[])[p_SCloo[], p_SClie...]
    # align iteratively
    oo_Ts_p = _PCL.alignPointCloudsLOOIters!(
      o_Ts_p, 
      p_SCs, 
      false, 3
    )
    o_Tloo_p[] = oo_Ts_p[1]
    o_Tlie_p[:] .= oo_Ts_p[2:end]
  end
  # udpate the cached memory pointers
  
  # do loo alignments
  # LEGACY, frames: object, subcloud, body:  o_PC = o_H_sc * sc_H_b * b_PC
  
  # build the cached object for use in the hot loop computations
  _ObjAffSubcCache(;
    p_SCloo,
    o_Tloo_p, # slightly different by fine alignment over coarse alignment from fct.ohat_T_p
    fc_lie_lbls,
    p_SClie,
    o_Tlie_p,
  )
end

function IncrementalInference.getSample(
  cf::CalcFactor{S},
) where {S <: ObjectAffordanceSubcloud}
  #
  M = getManifold(Pose3)
  e0 = ArrayPartition(zeros(3),diagm(ones(3)))
  
  # TBD consider adding a perturbation before alignment
  o_Tloo_p = cf.cache.o_Tloo_p[]
  
  # 
  o_Ts_p = typeof(o_Tloo_p)[o_Tloo_p, cf.cache.o_Tlie_p...]
  p_SCs = typeof(cf.cache.p_SCloo[])[cf.cache.p_SCloo[], cf.cache.p_SClie...]
  
  # all compute done once in preambleCache
  oo_Tloo_p = if length(o_Ts_p) < 2
    cf.cache.o_Tloo_p[]
  else
    _oo_Tloo_p, o_PClie, o_PCloo = _PCL.alignPointCloudLOO!(
      o_Ts_p,
      p_SCs,
      1;
      updateTloo=false
    )
    cf.cache.o_Tloo_p[] = _oo_Tloo_p
    _oo_Tloo_p
  end
  
  # FIXME, need better stochastic calculation and representative covariance result in strong unimodal cases -- see this `getMeasurementParametric``
  
  # return the transform from pose to object as manifold tangent element
  # TODO confirm expansion around e0, since PosePose factors expand around `q`
  return log(M, e0, oo_Tloo_p) # inv(M, oo_Tloo_p))
end

IIF.getMeasurementParametric(oas::ObjectAffordanceSubcloud) = error("Special case on ObjectAffordanceSubcloud, use lower dispatch `getMeasurementParametric(::DFGFactor{CCW{<:ObjectAffordanceSubcloud}})` instead.")
function IIF.getMeasurementParametric(foas::DFGFactor{<:CommonConvWrapper{<:ObjectAffordanceSubcloud}})
  @warn "Only artificial inverse covariance available for `getMeasurementParametric(::DFGFactor{CCW{<:ObjectAffordanceSubcloud}})`" maxlog=3
  M = getManifold(getFactorType(foas))
  e0 = ArrayPartition(SA[0;0;0.], SMatrix{3,3}([1 0 0; 0 1 0; 0 0 1.]))
  iΣ = diagm([0.2*ones(3); 10*ones(3)])
  o_T_p = IIF._getCCW(foas).dummyCache.o_Tloo_p[]
  # FIXME, not happy with inv on o_T_p -- OAS factor should read from pose to object???
  μ = vee(M, e0, log(M, e0, inv(M, o_T_p)))
  μ, iΣ
end

function (cf::CalcFactor{<:ObjectAffordanceSubcloud})(X,p,q)
  # copied from Pose3Pose3
  M = getManifold(Pose3)
  # work in the group
  q̂ = Manifolds.compose(M, p, exp(M, identity_element(M, p), X)) 
  #TODO allocalte for vee! see Manifolds #412, fix for AD
  # Xc = zeros(6)
  # vee!(M, Xc, q, log(M, q, q̂))
  # FIXME, should be tangent vector not coordinates -- likely part of ManOpt upgrade
  Xc = vee(M, q, log(M, q, q̂))
  return Xc
end


## =================================================================================
## Object Utils
## =================================================================================


function makePointCloudObjectAffordance(
  dfg::AbstractDFG,
  ovl::Symbol;
  align=:fine # or :coarse
)
  M = getManifold(Pose3)
  ohat_SCs = _PCL.PointCloud()

  for flb in ls(dfg, ovl)
    fc = getFactor(dfg, flb)
    p_SC = IIF._getCCW(fc).dummyCache.p_SCloo[]
    # p_SC = _PCL.getDataPointCloud(dfg, getVariableOrder(fc)[1], getFactorType(fc).p_PCloo_blobId; checkhash=false) |> _PCL.PointCloud
    o_T_p = if align == :fine
      @warn("Using preamble fine alignment from $flb")
      IIF._getCCW(fc).dummyCache.o_Tloo_p[]
    else
      getFactorType(dfg, flb).ohat_T_p
    end
    ohat_SC = _PCL.apply(M, o_T_p, p_SC)
    cat(
      ohat_SCs,
      ohat_SC;
      reuse = true
    )
  end

  return ohat_SCs
end


function generateObjectAffordanceFromWorld!(
  dfg::AbstractDFG,
  olb::Symbol,
  vlbs::AbstractVector{<:Symbol},
  w_BBobj::_PCL.AbstractBoundingBox, #GeoB.Rect3;
  solveKey::Symbol = :default,
  pcBlobLabel = r"PCLPointCloud2"
)
  # add the object variable
  addVariable!(dfg, olb, Pose3)
  # add the object affordance subcloud factors
  for vlb in vlbs
    # make sure PPE is set on this solveKey
    setPPE!(dfg, vlb, solveKey)
    # 
    p_BBo, ohat_T_p = _PCL.transformFromWorldToLocal(dfg, vlb, w_BBobj; solveKey)
    oas = Caesar.ObjectAffordanceSubcloud(;
      p_BBo,
      ohat_T_p,
      p_PCloo_blobId = getDataEntry(dfg, vlb, pcBlobLabel).id,
    )
    addFactor!(dfg, [vlb; olb], oas)
  end
  
  # necessary workaround for rebuilding factor cache with all OAS factors present on object
  for flb in ls(dfg, olb)
    IIF.rebuildFactorMetadata!(dfg, getFactor(dfg, flb); _blockRecursionGradients=true)
  end

  olb
end


##