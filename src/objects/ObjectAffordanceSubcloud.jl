# factor for mechanizing object affordances

struct _ObjAffSubcCache{C}
  """ we need a cached list of all other factors attached to the object variable, since this factor is the leave one out (loo). 
      `::Tuple{Factor,OtherVar,FineTuneTransform}` """
  looVariables::Vector{Tuple{Symbol,Symbol,ArrayPartition}} = Tuple{Symbol,Symbol,ArrayPartition}[]
  """ Current best cached estimate of the loo-aggregate object point cloud """
  looObjCloud::PointCloud{C} = PointCloud()
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
"""
Base.@kwdef struct ObjectAffordanceSubcloud{B} <: AbstractManifoldMinimize
  """ offset to where the bounding box for an object should be, given variable's local lidar scan """ 
  b_T_o::ArrayPartition = ArrayPartition(zeros(3), diagm(ones(3)))
  """ subcloud is selected by this mask from variable's point cloud """
  boundingMask::B
end




function IIF.preambleCache(
  ::AbstractDFG, 
  ::AbstractVector{<:DFGVariable}, 
  ::ObjectAffordanceSubcloud
)
  _ObjAffSubcCache()
end