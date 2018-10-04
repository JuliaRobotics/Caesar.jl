using Distributions

include("ParseUtils.jl")

function bearing(x1::Array{Float64,1}, x2::Array{Float64,1})::Float64
    return atan2(x2[2] - x1[2], x2[1] - x1[1])
end

function relative_bearing(pose::Array{Float64, 1}, point::Array{Float64, 1})::Float64
    pose2pointbearing = atan2(point[2] - pose[2], point[1] - pose[1])
    pose_bearing = pose[3]
    b = pose2pointbearing - pose_bearing
    if b < -pi
        b += 2.0*pi
    end
    if b >= pi
        b -= 2.0*pi
    end
    return b
end

function range_meas(x1::Array{Float64,1}, x2::Array{Float64,1})::Float64
    return norm(x2 - x1)
end

function rb_to_xy(meas::Array{Float64}, curr_pose::Array{Float64})::Array{Float64}
    x, y, w = curr_pose
    r, b = meas

    theta = w + b
    lm_x = x + r * cos(theta)
    lm_y = y + r * sin(theta)
    return [lm_x; lm_y]
end

function landmarks_in_range(pose::Array{Float64,1}, landmarks::Array{Float64}, sensor_max_range::Float64)::Array{Float64}
    lm_in_range = Array{Float64,2}(size(landmarks,1),0)
    for i in 1:size(landmarks,2)
        lm = landmarks[:,i]
        if range_meas(pose[1:2], lm[1:2]) < sensor_max_range
            lm_in_range = [lm_in_range lm]
        end
    end
    return lm_in_range
end

# Return all landmarks in field of view
# params:
#     pose::Array{Float64,1} estimated robot pose
#     landmarks::Array{Float64} array of landmarks to test
#     sensor_max_range::Float64 maximum sensor range (m)
#     sensor_fov::Float64 total sensor field of view (rad)
# return:
#     Array{Float64} array of landmarks within the field of view of the robot from the estimated pose
function landmarks_in_fov(pose::Array{Float64,1}, landmarks::Array{Float64}, sensor_max_range::Float64, sensor_fov::Float64)::Array{Float64}
    lm_in_fov = Array{Float64,2}(size(landmarks,1),0)
    for i in 1:size(landmarks,2)
        lm = landmarks[:,i]
        if range_meas(pose[1:2], lm[1:2]) < sensor_max_range && abs(relative_bearing(pose[1:3], lm[1:2])) < sensor_fov/2.0
            lm_in_fov = [lm_in_fov lm]
        end
    end
    return lm_in_fov
end

function measure(pose::Array{Float64,1},
                 landmarks::Array{Float64},
                 sensor_max_range::Float64,
                 bearing_noise::Float64,
                 range_noise::Float64)::Array{Float64}

    meas = Array{Float64,2}(2,0)
    lm_in_range = landmarks_in_range(pose, landmarks, sensor_max_range)
    if isempty(lm_in_range)
        return []
    end
    for i in 1:size(lm_in_range, 2)
        lm = lm_in_range[:,i]
        z = range_meas(pose[1:2], lm) + rand(Normal(0, range_noise))
        b = bearing(pose[1:2], lm) - pose[3] + rand(Normal(0, bearing_noise))

        # Correction for bearing?
        if b < 0.0
            b = b + 2*pi
        end
        if b > 2*pi
            b = b - 2*pi
        end

        meas = [meas [z; b]]
    end
    return meas
end

# transform coordinate p2 (originally relative to p1)
# into global coordinate frame [0.0; 0.0; 0.0]
function transform_relative_to_global(p1, p2)
    x1, y1, w1 = p1
    total_w = p2[3] + p1[3]
    R = [cos(w1) -sin(w1); sin(w1) cos(w1)]
    A = [R [x1; y1]; 0.0 0.0 1.0]
    p2 = [p2[1:2]; 1.0]
    global_p2 = A*p2
    global_p2[3] = total_w
    return global_p2
end

# transform coordinate p2 (originally in global frame)
# into relative coordinate frame p1
function transform_global_to_relative(p1, p2)
    x1, y1, w1 = p1
    dw = p2[3] - p1[3]
    R = [cos(w1) -sin(w1); sin(w1) cos(w1)]
    A = [R [x1; y1]; 0.0 0.0 1.0]
    p2 = [p2[1:2]; 1.0]
    rel_p2 = inv(A)*p2
    rel_p2[3] = dw
    return rel_p2
end

function measure_semantic(pose::Array{Float64,1},
                          landmarks::Array{Float64},
                          sensor_max_range::Float64,
                          sensor_fov::Float64,
                          bearing_noise::Float64,
                          range_noise::Float64,
                          n_classes::Int64)::Array{Float64}

    meas = Array{Float64,2}(2 + n_classes,0)
    @show lm_in_fov = landmarks_in_fov(pose, landmarks, sensor_max_range, sensor_fov)
    @show size(lm_in_fov)
    if isempty(lm_in_fov)
        return []
    end
    for i in 1:size(lm_in_fov,2)
        lm = lm_in_fov[:,i]
        z = range_meas(pose[1:2], lm[1:2]) + rand(Normal(0, range_noise))
        b = bearing(pose[1:2], lm[1:2]) - pose[3] + rand(Normal(0, bearing_noise))

        # Here the confusion matrix is set up for a classifier with 90% accuracy, independent of class
        # This can be made a parameter in future
        p = ones(n_classes) * (0.1 / (n_classes - 1))
        p[convert(Int64, lm[3])] = 0.9
        pred = rand(Categorical(p)) # Generate a random sample from confusion matrix probabilities
        s = ones(n_classes) * (0.1 / (n_classes - 1))
        s[pred] = 0.9

        # Correction for bearing
        if b < -pi
            b = b + 2*pi
        end

        if b >= pi
            b = b - 2*pi
        end

        meas = [meas [z; b; s]]
    end
    return meas
end

function run_sim(poses, landmarks, sensor_max_range, sensor_fov, range_noise, bearing_noise, odom_noise, n_classes; save_data=false)
    sbr_data = Array{Float64, 2}(1 + 2 + n_classes, 0)
    odom_data = Array{Float64, 2}(5, 0)
    for i in 1:size(poses,2)
        pose = poses[:,i]
        if i > 1
            previous_pose = poses[:,i-1]
            rel_xyz = transform_global_to_relative(previous_pose, pose + rand(MvNormal(zeros(3), odom_noise)))
            odom_meas = rel_xyz
            odom_meas = [i - 1; i; odom_meas]
            odom_data = [odom_data odom_meas]
        end
        sbr_meas = measure_semantic(pose, landmarks, sensor_max_range, sensor_fov, range_noise, bearing_noise, n_classes)
        if !isempty(sbr_meas)
          sbr_meas = [i; sbr_meas]
          sbr_data = [sbr_data sbr_meas]
        end

    end

    # Write results to files
    # Could param out a data directory
    if save_data
        stamp = Dates.format(Dates.now(), "yyyy-mm-dd-HH-MM-SS")
        @show stamp
        write_poses_landmarks(poses, landmarks, string("../data/poses_", stamp, ".txt"), string("../data/landmarks_", stamp, ".txt"))
        write_measurements(odom_data, string("../data/odom_", stamp, ".txt"))
        write_measurements(sbr_data, string("../data/sbr_", stamp, ".txt"))
    end
end

# Measurement parameters
sensor_max_range = 3.5
sensor_fov = 2.0*pi/3.0
landmark_sample_noise = 0.1
range_model_noise = 0.1
bearing_model_noise = 0.025
odom_noise = 0.001*eye(3)
n_classes = 2

# Set up landmarks; would be cool to randomly generate these
landmarks = [[1.0; 1.0; 1] [1.0; -1.0; 2] [3.0; 1.0; 2] [4.0; -1.0; 2] [6.0; 1.0; 1] [6.0; -1.0; 1] [8.0; -1.0; 2] [9.0; 1.0; 1] [11.0; -1.0; 2] [11.0; 2.0; 1] [11.0; 4.0; 2] [11.0; 5.0; 1] [11.0; 7.0; 2] [11.0; 9.0; 1] [11.0; 11.0; 2] [9.0; 3.0; 2] [9.0; 4.0; 1] [9.0; 6.0; 1] [9.0; 9.0; 2] [8.0; 9.0; 1] [8.0; 11.0; 2] [7.0; 11.0; 1] [5.0; 9.0; 2] [5.0; 11.0; 2] [4.0; 11.0; 1] [3.0; 9.0; 1] [1.0; 6.0; 1] [-1.0; 10.0; 2] [-1.0; 5.0; 1]]

# True robot poses; would be cool to randomly generate these
poses = [[0.0; 0.0; 0.0] [2.0; 0.0; 0.0] [4.0; 0.0; 0.0] [6.0; 0.0; 0.0] [8.0; 0.0; 0.0] [10.0; 0.0; pi/2] [10.0; 2.0; pi/2] [10.0; 4.0; pi/2] [10.0; 6.0; pi/2] [10.0; 8.0; pi/2] [10.0; 10.0; pi] [8.0; 10.0; pi] [6.0; 10.0; pi] [4.0; 10.0; pi] [2.0; 10.0; pi] [0.0; 10.0; 3*pi/2] [0.0; 8.0; 3*pi/2] [0.0; 6.0; 3*pi/2] [0.0; 4.0; 3*pi/2] [0.0; 2.0; 3*pi/2] [0.0; 0.0; 0.0]]

# Run the simulation and save data files
# run_sim(poses, landmarks, sensor_max_range, sensor_fov, range_model_noise, bearing_model_noise, odom_noise, n_classes, save_data=true)

