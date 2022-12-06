# factor for mechanizing object affordances
Base.@kwdef struct _ObjAffSubcCache{T}
  """ cache dict of all pointclouds connected to similar factors of this object variable """
  relativePointClouds::Dict{Symbol,T}
  """ this factor's subcloud -- i.e. lieSubcloud or liePts (leave-in-element) """
  lieObjCloud::_PCL.PointCloud{<:Any}
  """ current best transform estimate for this lie subcloud to the object reference frame """
  o_Tlie_sc::typeof(ArrayPartition(SA[0.;0.;0.],SMatrix{3,3}(randn(3,3)))) = ArrayPartition(SA[0.;0.;0.],SMatrix{3,3}(diagm([1;1;1.])))
  """ we need a cached list of all other factors attached to the object variable, since this factor is the leave one out (loo). 
      `::Tuple{Factor,OtherVar,FineTuneTransform}` """
  looVariables::Vector{Tuple{Symbol,Symbol,<:ArrayPartition}} = Vector{Tuple{Symbol,Symbol,typeof(ArrayPartition(SA[0.;0.;0.],SMatrix{3,3}(diagm([1;1;1.]))))}}()
  """ Current best cached estimate of the loo-aggregate object point cloud """
  looObjCloud::_PCL.PointCloud{<:Any} = PointCloud()
end


"""
    $TYPEDEF

Objects appear in lidar scans (bigger or smaller than the vehicle).  This factor masks a subcloud from the
variable point cloud and then tries to use that subcloud as a navigation affordance.  

Valid objects examples:
- A potplant standing in the room.
- An entire tree standing outside the room.
- The room itself around the vehicle.

Notes
- Bounding box definition is only local to this variables lidar scan, therefore not 
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
  boundingMask::B = GeoB.Rect([GeoB.Point(0,0,0.),GeoB.Point(1,1,1.)])
  """ standard entry blob label where to find the point clouds -- 
  forced to use getData since factors need to react when other factors are 
  added to the same object variable later by the user.
  FIXME: there is a hack, should not be using Serialization.deserialize on a PCLPointCloud2, see Caesar.jl#921 """
  pc_datalabel::String = "PCLPointCloud2"
end

## TODO maybe overload addFactor! for ::ObjectAffordanceSubcloud so that when new factors are added 
# to the same object, each of the preambleCache functions can rerun, and thereby update with new loo 
# info, alternatively this update needs to be detect in the getSample functions which may not have 
# access to all the heavy lift data wrangling functions.  Solution is to readd Factors on loo list.

function IncrementalInference.preambleCache(
  dfg::AbstractDFG, 
  fvars::AbstractVector{<:DFGVariable}, 
  fct::ObjectAffordanceSubcloud
)
  # get all pose variables connected to this object
  # TODO, confirm fvars[2] works under multihypo use
  lievlb = getLabel(fvars[1])
  olbl = getLabel(fvars[2])
  aflb = ls(dfg,olbl)
  avlbs = union(ls.(dfg,aflb)...)
  loovlbs = setdiff(avlbs, [lievlb;])
  
  # the the full variable point clouds from all variables currently connected to the object variable
  # TODO use concrete types
  _PCT(::PC) where {PC <: _PCL.PointCloud} = PC
  PCT = _PCT(_PCL.PointCloud())
  relativePointClouds = Dict{Symbol,PCT}()
  objPCs = Dict{Symbol,PCT}()
  
  for lbl in avlbs
    # fetch and cache the full pose point cloud given a starndard data label
    relativePointClouds[lbl] = getDataPointCloud(dfg,lbl,fct.pc_datalabel) |> _PCL.PointCloud
    # extract just those subcloud points (assumed to now represent the object observation from this pose variable)
    objPCs[lbl] = _PCL.getSubcloud(relativePointClouds[lbl], fct.boundingMask)
  end
    
  # finalize object point clouds for cache
  lieObjCloud = objPCs[lievlb]

  # do loo alignments
  # NOTE, frames: object, subcloud, body:  o_PC = o_H_sc * sc_H_b * b_PC
  looObjCloud = _PCL.PointCloud()

  # cache summary for the loo variables
  e0 = ArrayPartition(SA[0.;0.;0.],SMatrix{3,3}(diagm(ones(3))))
  APT = typeof(e0)
  looVariables = Vector{Tuple{Symbol,Symbol,APT}}()
  for loov in loovlbs
    specFcts = intersect(aflb,ls(dfg,loov))
    # must be OAS factor
    filter!(l->getFactorType(dfg,l) isa ObjectAffordanceSubcloud, specFcts)
    push!(
      looVariables, 
      ( 
        specFcts[1], # TODO, MAKER_asfwe, allow more than one OAS solution to work in parallel on same object variable
        loov,
        e0
      )
    )
  end
  
  # build the cached object for use in the hot loop computations
  _ObjAffSubcCache(;
    relativePointClouds,
    lieObjCloud,
    looVariables,
    looObjCloud
  )
end

function IncrementalInference.getSample(
  cf::CalcFactor{S},
) where {S <: ObjectAffordanceSubcloud}
  #
  M = getManifold(Pose3)
  e0 = identity_element(M)
  
  # TODO see if new factors have been added to the object variable, therefore requiring 
  # update of the cached loo registers
  
  
  # only two variables
  pose = getVariable(cf.fullvariables[1])
  objv = getVariable(cf.fullvariables[2])
  
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
  
  # convert SE affine Homgraphy to manifold element 
  p_P_q = ArrayPartition(p_Hicp_phat[1:end-1,end],p_Hicp_phat[1:end-1,1:end-1])
  
  # return the transform from pose to object as manifold tangent element
  # TODO confirm expansion around e0, since PosePose factors expand around `q`
  return log(M, e0, p_P_q)
end