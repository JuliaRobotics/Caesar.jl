julia -O3 LCM_SLAM_client.jl --dbg -i 10000 -s 0.2 --kappa_odo 10 --magStdDeg 10.0
julia -O3 LCM_SLAM_client.jl --dbg -i 40000 -s 0.13 --kappa_odo 10.0 --magStdDeg 2.0 --stride_range 3
