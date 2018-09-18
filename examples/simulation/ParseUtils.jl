# ParseUtils.jl
#
# For now, a set of wrappers for Julia readdlm and writedlm
#
# Kevin Doherty 2018

# param pose_file: path to ground truth poses (ordered)
#    formatted as (x, y, theta)
# param lm_file: path to ground truth landmarks (ordered)
#    formatted as (x, y, class)
# expected return: pose array and landmark array
function read_poses_landmarks(pose_fname, lm_fname)::Tuple{Array{Float64}, Array{Float64}}
    poses = readdlm(pose_fname, ',')
    landmarks = readdlm(lm_fname, ',')
    return (poses, landmarks)
end

function write_poses_landmarks(poses, landmarks, pose_fname, lm_fname)
    writedlm(pose_fname, poses, ",")
    writedlm(lm_fname, landmarks, ",")
end

function read_measurements(fname)::Array{Float64}
    meas = readdlm(fname, ',')
    return meas
end

function write_measurements(meas, fname)
    writedlm(fname, meas, ",")
end
