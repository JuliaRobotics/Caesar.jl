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
  # """ cache dict of all pointclouds connected to similar factors of this object variable """
  # relativePointClouds::Dict{Symbol,T}
  # """ this factor's subcloud -- i.e. lieSubcloud or liePts (leave-in-element) """
  # lieObjCloud::_PCL.PointCloud{<:Any}
  # """ current best transform estimate for this lie subcloud to the object reference frame """
  # o_Tlie_sc::typeof(ArrayPartition(SA[0.;0.;0.],SMatrix{3,3}(randn(3,3)))) = ArrayPartition(SA[0.;0.;0.],SMatrix{3,3}(diagm([1;1;1.])))
  # """ we need a cached list of all other factors attached to the object variable, since this factor is the leave one out (loo). 
  #     `::Tuple{Factor,OtherVar,FineTuneTransform}` """
  # looVariables::Vector{Tuple{Symbol,Symbol,<:ArrayPartition}} = Vector{Tuple{Symbol,Symbol,typeof(ArrayPartition(SA[0.;0.;0.],SMatrix{3,3}(diagm([1;1;1.]))))}}()
  # """ Current best cached estimate of the loo-aggregate object point cloud """
  # looObjCloud::_PCL.PointCloud{<:Any} = PointCloud()
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
- TODO, MAKER_asfwe, allow more than one OAS solution to work in parallel on same object variable
"""
Base.@kwdef struct ObjectAffordanceSubcloud{B} <: AbstractManifoldMinimize
  # """ body to object offset (or object in body frame) to where the bounding box for an object should be, given variable's local lidar scan """ 
  # b_T_o::ArrayPartition = ArrayPartition(zeros(3), diagm(ones(3)))
  # SOMEKINDOFOBJPCORIGIN, o_H_sc
  """ subcloud is selected by this mask from the variable's point cloud """
  p_BBo::B = GeoB.Rect([GeoB.Point(0,0,0.),GeoB.Point(1,1,1.)])
  """ standard entry blob label where to find the point clouds -- 
  forced to use getData since factor cache needs to update when other factors are 
  added to the same object variable later by the user.  See [`IIF.rebuildFactorMetadata!`](@ref).
  FIXME: there is a hack, should not be using Serialization.deserialize on a PCLPointCloud2, see Caesar.jl#921 """
  p_PCloo_blobId::UUID
  # pc_datalabel::String = "PCLPointCloud2"
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
  @info "WHY X1" loovlb fct.p_PCloo_blobId
  p_PC = _PCL.getDataPointCloud(dfg, loovlb, fct.p_PCloo_blobId; checkhash=false) |> _PCL.PointCloud
  _p_SC = _PCL.getSubcloud(p_PC, fct.p_BBo)
  p_SCloo = Ref(_p_SC)
  o_Tloo_p = Ref(e0)
  
  # define the LIE elements
  for (idx,liev) in enumerate(lievlbs)
    @show specFcts = intersect(aflb,ls(dfg,liev))
    # must be OAS factor
    filter!(l->@show(getFactorType(dfg,l)) isa ObjectAffordanceSubcloud, specFcts)
    # filtering from all factors, skip this variable if not part of this object affordance 
    0 === length(specFcts) ? continue : nothing
    fc_lb = specFcts[1]
    # collect factor labels
    push!(fc_lie_lbls, fc_lb)
    # find self (loo) number from factor list
    A, B = lsf(dfg, fc_lb), getLabel.(fvars)
    0 === length(setdiff(A,B)) ? loo_idx = idx : nothing
    # collect list of all subclouds
    p_SC = _PCL.getDataSubcloudLocal(dfg, liev, fct.p_BBo, r"PCLPointCloud2"; checkhash=false)
    push!(p_SClie, p_SC)
    # collect list of all pose to object transforms
    # FIXME, should start with current best guess provided by user
    push!(o_Tlie_p, e0)
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
    o_Tloo_p,
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
  
  # TODO see if new factors have been added to the object variable, therefore requiring 
  # update of the cached loo registers
  
  # only two variables
  # pose = getVariable(cf.fullvariables[1])
  # objv = getVariable(cf.fullvariables[2])
  
  # Do a loo alignment against best aggregate lie clouds

  # # get the aggregate loo subcloud from other (already aligned) factor subclouds 
  # p_looPts = 
  
  # # get the lie subcloud from this factor subcloud
  # q_liePts = 
  
  # # do of lie against loo pts alignment
  # p_Hicp_phat, Hpts_mov, status = _PCL.alignICP_Simple(
  #   p_looPts, # fixed cloud
  #   q_liePts; # transforming cloud
  #   verbose=false,
  #   max_iterations = 25,
  #   correspondences = 500,
  #   neighbors = 50
  # )
  
  # # convert SE affine Homgraphy to manifold element 
  # p_P_q = ArrayPartition(p_Hicp_phat[1:end-1,end],p_Hicp_phat[1:end-1,1:end-1])
  
  # return the transform from pose to object as manifold tangent element
  # TODO confirm expansion around e0, since PosePose factors expand around `q`
  return log(M, e0, e0) # p_P_q)
end


##