#=
LCM Server: an LCM interface to Caesar.jl

=#

using Caesar, RoME
using TransformUtils, Rotations, CoordinateTransformations
using Distributions
using PyCall, PyLCM
# using LibBSON
# using CloudGraphs # for sorryPedro

function gen_bindings()
    @show lcmtpath = joinpath(dirname(@__FILE__),"lcmtypes")
    run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/rome_point_cloud_t.lcm`)
    run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/rome_pose_node_t.lcm`)
    run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/rome_pose_pose_nh_t.lcm`)
    run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/rome_pose_pose_xyh_t.lcm`)
    run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/rome_prior_zpr_t.lcm`)
    println("Adding lcmtypes dir to Python path: $(lcmtpath)")
    unshift!(PyVector(pyimport("sys")["path"]),lcmtpath)
end

println("[Caesar.jl] (re)generating LCM bindings")
gen_bindings()

println("[Caesar.jl] Importing LCM message types")
@pyimport rome


## ========================================================================== ##


wTx = Vector{AffineMap}(5)
wTx[1] = Translation(8.7481, 7.51, 4.5662) ∘ LinearMap(Quat(0.9939, -0.0280019, -0.00824962, -0.106355))
wTx[2] = Translation(9.43, 11.0879, 4.9) ∘ LinearMap(Quat(0.992387, 0.0575732, -0.00698004, -0.108645))
wTx[3] = Translation(9.94, 15.1484, 4.8792) ∘ LinearMap(Quat(0.997597, 0.0563353, -0.0115003, -0.0386678))
wTx[4] = Translation(9.93104, 14.5831, 5.8) ∘ LinearMap(Quat(0.997626, 0.0336346, -0.011705, -0.0589371))
wTx[5] = Translation(9.12, 10.1615, 5.84927) ∘ LinearMap(Quat(0.998351, 0.0362801, -0.00973871, -0.0434122))


wTx1 = convert(SE3, wTx[1])
wTx2 = convert(SE3, wTx[2])
wTx3 = convert(SE3, wTx[3])
wTx4 = convert(SE3, wTx[4])
wTx5 = convert(SE3, wTx[5])
# get rotation to world frame for local level
wEx1 = convert(Euler, wTx1.R);  wEx1.Y = 0.0;
wRlx1 = SE3(zeros(3), wEx1)
wEx2 = convert(Euler, wTx2.R);  wEx2.Y = 0.0;
wRlx2 = SE3(zeros(3), wEx2)
wEx3 = convert(Euler, wTx3.R);  wEx3.Y = 0.0;
wRlx3 = SE3(zeros(3), wEx3)
wEx4 = convert(Euler, wTx4.R);  wEx4.Y = 0.0;
wRlx4 = SE3(zeros(3), wEx4)
wEx5 = convert(Euler, wTx5.R);  wEx5.Y = 0.0;
wRlx5 = SE3(zeros(3), wEx5)
# Odometries
wRlx1Tx2 = wRlx1 * (wTx1\wTx2)
wRlx2Tx3 = wRlx2 * (wTx2\wTx3)
wRlx3Tx4 = wRlx3 * (wTx3\wTx4)
wRlx4Tx5 = wRlx4 * (wTx4\wTx5)

vEx1_2 = veeEuler(wRlx1Tx2)
vEx2_3 = veeEuler(wRlx2Tx3)
vEx3_4 = veeEuler(wRlx3Tx4)
vEx4_5 = veeEuler(wRlx4Tx5)

XYH1_2 = [vEx1_2[1], vEx1_2[2], vEx1_2[6]]
XYH2_3 = [vEx2_3[1], vEx2_3[2], vEx2_3[6]]
XYH3_4 = [vEx3_4[1], vEx3_4[2], vEx3_4[6]]
XYH4_5 = [vEx4_5[1], vEx4_5[2], vEx4_5[6]]

zrp2 = veeEuler(wTx2)[[3;4;5]]
zrp3 = veeEuler(wTx3)[[3;4;5]]
zrp4 = veeEuler(wTx4)[[3;4;5]]
zrp5 = veeEuler(wTx5)[[3;4;5]]



lcm_node = LCM()


# send the first pose
msg = rome.pose_node_t()
msg[:utime] = 0
msg[:id] = 1
msg[:mean_dim] = 7
q1 = convert(Quaternion, wTx1.R)
msg[:mean] = Float64[wTx1.t..., q1.s, q1.v...]
msg[:covar_dim] = 6
msg[:covar] = Float64[0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
publish(lcm_node, "ROME_POSES", msg)
sleep(0.01)



# send the second pose
msg = rome.pose_node_t()
msg[:utime] = 0
msg[:id] = 2
msg[:mean_dim] = 7
q2 = convert(Quaternion, wTx2.R)
msg[:mean] = Float64[wTx2.t..., q2.s, q2.v...]
msg[:covar_dim] = 6
msg[:covar] = Float64[0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
publish(lcm_node, "ROME_POSES", msg)
sleep(0.01)

msg = rome.prior_zpr_t()
msg[:utime] = 0
msg[:id] = 2
msg[:z] = zrp2[1]
msg[:roll] = zrp2[2]
msg[:pitch] = zrp2[3]
msg[:var_z] = 0.01
msg[:var_pitch] = 0.001
msg[:var_roll] = 0.001
publish(lcm_node, "ROME_PARTIAL_ZPR", msg)
sleep(0.01)

msg = rome.pose_pose_xyh_t()
msg[:utime] = 0
msg[:node_1_utime] = 0
msg[:node_1_id] = 1
msg[:node_2_utime] = 0
msg[:node_2_id] = 2
msg[:delta_x] = XYH1_2[1]
msg[:delta_y] = XYH1_2[2]
msg[:delta_yaw] = XYH1_2[3]
msg[:var_x] = 0.001
msg[:var_y] = 0.001
msg[:var_yaw] = 0.001
publish(lcm_node, "ROME_PARTIAL_XYH", msg)




# send the third pose
msg = rome.pose_node_t()
msg[:utime] = 0
msg[:id] = 3
msg[:mean_dim] = 7
q3 = convert(Quaternion, wTx3.R)
msg[:mean] = Float64[wTx3.t..., q3.s, q3.v...]
msg[:covar_dim] = 6
msg[:covar] = Float64[0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
publish(lcm_node, "ROME_POSES", msg)
sleep(0.01)

msg = rome.prior_zpr_t()
msg[:utime] = 0
msg[:id] = 3
msg[:z] = zrp3[1]
msg[:roll] = zrp3[2]
msg[:pitch] = zrp3[3]
msg[:var_z] = 0.01
msg[:var_pitch] = 0.001
msg[:var_roll] = 0.001
publish(lcm_node, "ROME_PARTIAL_ZPR", msg)
sleep(0.01)

msg = rome.pose_pose_xyh_t()
msg[:utime] = 0
msg[:node_1_utime] = 0
msg[:node_1_id] = 2
msg[:node_2_utime] = 0
msg[:node_2_id] = 3
msg[:delta_x] = XYH2_3[1]
msg[:delta_y] = XYH2_3[2]
msg[:delta_yaw] = XYH2_3[3]
msg[:var_x] = 0.001
msg[:var_y] = 0.001
msg[:var_yaw] = 0.001
publish(lcm_node, "ROME_PARTIAL_XYH", msg)











# send the forth pose
msg = rome.pose_node_t()
msg[:utime] = 0
msg[:id] = 4
msg[:mean_dim] = 7
q4 = convert(Quaternion, wTx4.R)
msg[:mean] = Float64[wTx4.t..., q4.s, q4.v...]
msg[:covar_dim] = 6
msg[:covar] = Float64[0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
publish(lcm_node, "ROME_POSES", msg)
sleep(0.01)

msg = rome.prior_zpr_t()
msg[:utime] = 0
msg[:id] = 4
msg[:z] = zrp4[1]
msg[:roll] = zrp4[2]
msg[:pitch] = zrp4[3]
msg[:var_z] = 0.01
msg[:var_pitch] = 0.001
msg[:var_roll] = 0.001
publish(lcm_node, "ROME_PARTIAL_ZPR", msg)
sleep(0.01)

msg = rome.pose_pose_xyh_t()
msg[:utime] = 0
msg[:node_1_utime] = 0
msg[:node_1_id] = 3
msg[:node_2_utime] = 0
msg[:node_2_id] = 4
msg[:delta_x] = XYH3_4[1]
msg[:delta_y] = XYH3_4[2]
msg[:delta_yaw] = XYH3_4[3]
msg[:var_x] = 0.001
msg[:var_y] = 0.001
msg[:var_yaw] = 0.001
publish(lcm_node, "ROME_PARTIAL_XYH", msg)










# send the fifth pose
msg = rome.pose_node_t()
msg[:utime] = 0
msg[:id] = 5
msg[:mean_dim] = 7
q5 = convert(Quaternion, wTx4.R)
msg[:mean] = Float64[wTx4.t..., q5.s, q5.v...]
msg[:covar_dim] = 6
msg[:covar] = Float64[0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
publish(lcm_node, "ROME_POSES", msg)
sleep(0.01)

msg = rome.prior_zpr_t()
msg[:utime] = 0
msg[:id] = 5
msg[:z] = zrp5[1]
msg[:roll] = zrp5[2]
msg[:pitch] = zrp5[3]
msg[:var_z] = 0.01
msg[:var_pitch] = 0.001
msg[:var_roll] = 0.001
publish(lcm_node, "ROME_PARTIAL_ZPR", msg)
sleep(0.01)

msg = rome.pose_pose_xyh_t()
msg[:utime] = 0
msg[:node_1_utime] = 0
msg[:node_1_id] = 4
msg[:node_2_utime] = 0
msg[:node_2_id] = 5
msg[:delta_x] = XYH4_5[1]
msg[:delta_y] = XYH4_5[2]
msg[:delta_yaw] = XYH4_5[3]
msg[:var_x] = 0.001
msg[:var_y] = 0.001
msg[:var_yaw] = 0.001
publish(lcm_node, "ROME_PARTIAL_XYH", msg)















#
# function handle_poses!(slam::SLAMWrapper,
#                        message_data)
#
#     message = rome.pose_node_t[:decode](message_data)
#     id = message[:id]
#     println("[Caesar.jl] Received pose message for x$(id)")
#
#     mean = message[:mean]
#     covar = message[:covar]
#     t = [mean[1], mean[2], mean[3]]
#     qw = mean[4]
#     qxyz = [mean[5], mean[6], mean[7]]
#     q = Quaternion(qw,qxyz) # why not a (w,x,y,z) constructor?
#     pose = SE3(t,q)
#     euler = Euler(q)
#
#     node_label = Symbol("x$(id)")
#     nothing
# end
#
# function handle_priors!(slam::SLAMWrapper,
#                          message_data)
#
#
#     message = rome.prior_zpr_t[:decode](message_data)
#     id = message[:id]
#     println("[Caesar.jl] Adding prior on RPZ to x$(id)")
#
#     @show node_label = Symbol("x$(id)")
#
#     @show message[:z]
#     @show message[:pitch]
#     @show message[:roll]
#
#     @show message[:var_z]
#     @show message[:var_pitch]
#     @show message[:var_roll]
#     nothing
# end
#
#
# function handle_partials!(slam::SLAMWrapper,
#                          message_data)
#     # add XYH factor
#
#     message = rome.pose_pose_xyh_t[:decode](message_data)
#
#     origin_id = message[:node_1_id]
#     destination_id = message[:node_2_id]
#     origin_label = Symbol("x$(origin_id)")
#     destination_label = Symbol("x$(destination_id)")
#
#     println("[Caesar.jl] Adding XYH odometry constraint betwee (x$(origin_id), x$(destination_id))")
#
#     @show message[:delta_x]
#     @show message[:delta_y]
#     @show message[:delta_yaw]
#
#     @show message[:var_x]
#     @show message[:var_y]
#     @show message[:var_yaw]
#
#     @show origin_label, destination_label
#     nothing
# end
#
#
#
# # Now test and evaluate if the lcm log is in fact correct
#
# lcm_pose_handler = (channel, message_data) -> handle_poses!(slam_client, message_data )
# lcm_odom_handler = (channel, message_data) -> handle_partials!(slam_client, message_data )
# lcm_prior_handler = (channel, message_data) -> handle_priors!(slam_client, message_data )





#
