##
using PyCall
using Manifolds

py"""
import numpy as np
import open3d as o3d

def createpcd(xy):
  pcd = o3d.geometry.PointCloud()
  pcd.points = o3d.utility.Vector3dVector(xy)
  return pcd
"""

function _open3d_registration_icp_qHp(
  source::AbstractArray,
  target::AbstractArray;
  initial_H = Matrix(1.0I,4,4),#py"np.identity"(4) # Initial transformation for ICP
  th_distance = 3.0, # The threshold distance used for searching correspondences (closest points between clouds). I'm setting it to 10 cm.
  # Define the type of registration:
  type = py"o3d.pipelines.registration.TransformationEstimationPointToPoint"(false), # "False" means rigid transformation, scale = 1
  # type = py"o3d.pipelines.registration.TransformationEstimationPointToPlane"(false), # "False" means rigid transformation, scale = 1
  # Define the number of iterations:
  max_iterations::Int = 100,
  iterations = py"o3d.pipelines.registration.ICPConvergenceCriteria"(max_iteration = max_iterations),
)
  source_pc = py"createpcd"(source)
  target_pc = py"createpcd"(target)
  
  # Do the registration:
  result = py"o3d.pipelines.registration.registration_icp"(source_pc, target_pc, th_distance, initial_H, type, iterations)

  #TODO don't know how to cleanly get data from python
  qHp_3d = hcat(collect.(collect(result.transformation))...)'
  
  # the returned value is from q to p. 
  q_H_p = [qHp_3d[1:3,1:3] qHp_3d[1:3,4]; [0 0 0 1]]

  return q_H_p, result.fitness, result.inlier_rmse, result.correspondence_set
end

function open3d_registration_2d_icp(
  source::AbstractArray,
  target::AbstractArray;
  initial_H = Matrix(1.0I,4,4),#py"np.identity"(4) # Initial transformation for ICP
  th_distance = 3.0, # The threshold distance used for searching correspondences (closest points between clouds). I'm setting it to 10 cm.
  # Define the type of registration:
  type = py"o3d.pipelines.registration.TransformationEstimationPointToPoint"(false), # "False" means rigid transformation, scale = 1
  # Define the number of iterations:
  max_iterations::Int = 100,
  iterations = py"o3d.pipelines.registration.ICPConvergenceCriteria"(max_iteration = max_iterations),
  M = SpecialEuclidean(2)
)

  q_H_p_, fitness, inlier_rmse, correspondence_set = _open3d_registration_icp_qHp(source, target; initial_H, th_distance, type, iterations)
  
  q_H_p = [q_H_p_[1:2,1:2] q_H_p_[1:2,4]; [0 0 1]]
  p_H_q = inv(M, q_H_p)

  return p_H_q, fitness, inlier_rmse, correspondence_set
  
end

@deprecate open3d_registration_icp(w...;kw...) open3d_registration_2d_icp(w...;kw...)


function open3d_registration_3d_icp(
  source::AbstractArray,
  target::AbstractArray;
  initial_H = Matrix(1.0I,4,4),#py"np.identity"(4) # Initial transformation for ICP
  th_distance = 3.0, # The threshold distance used for searching correspondences (closest points between clouds). I'm setting it to 10 cm.
  # Define the type of registration:
  type = py"o3d.pipelines.registration.TransformationEstimationPointToPoint"(false), # "False" means rigid transformation, scale = 1
  # Define the number of iterations:
  max_iterations::Int = 100,
  iterations = py"o3d.pipelines.registration.ICPConvergenceCriteria"(max_iteration = max_iterations),
  M = SpecialEuclidean(3)
)
  q_H_p, fitness, inlier_rmse, correspondence_set = _open3d_registration_icp_qHp(source, target; initial_H, th_distance, type, iterations)
  p_H_q = inv(M, q_H_p)
  
  return p_H_q, fitness, inlier_rmse, correspondence_set
end