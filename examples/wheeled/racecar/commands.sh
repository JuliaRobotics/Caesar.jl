#!/bin/bash

# julia102 -O 3 -p 4 apriltag_and_zed_slam.jl --folder_name "labrun8" --failsafe
# julia102 -O 3 -p 4 apriltag_and_zed_slam.jl --folder_name "labrun8" --failsafe --focalscale 1.005
# julia102 -O 3 -p 4 apriltag_and_zed_slam.jl --folder_name "labrun8" --failsafe --focalscale 0.995
# julia102 -O 3 -p 6 apriltag_and_zed_slam.jl --folder_name "labrun8" --failsafe > out8



racecarslamdefault_1() {
    julia -O 3 -p 8 apriltag_and_zed_slam.jl --folder_name "labrun1" $*
}
racecarslamdefault_2() {
    julia -O 3 -p 8 apriltag_and_zed_slam.jl --folder_name "labrun2" $*
}
racecarslamdefault_3() {
    julia -O 3 -p 8 apriltag_and_zed_slam.jl --folder_name "labrun3" $*
}
racecarslamdefault_4() {
    julia -O 3 -p 8 apriltag_and_zed_slam.jl --folder_name "labrun4" $*
}
racecarslamdefault_5() {
    julia -O 3 -p 8 apriltag_and_zed_slam.jl --folder_name "labrun5" $*
}
racecarslamdefault_6() {
    julia -O 3 -p 8 apriltag_and_zed_slam.jl --folder_name "labrun6" $*
}
racecarslamdefault_7() {
    julia -O 3 -p 8 apriltag_and_zed_slam.jl --folder_name "labrun7" $*
}
racecarslamdefault_8() {
    julia -O 3 -p 8 apriltag_and_zed_slam.jl --folder_name "labrun8" $*
}

racecarall() {
  racecarslamdefault_1 $* &
  racecarslamdefault_2 $* &
  racecarslamdefault_3 $* &
  racecarslamdefault_4 $* &
  racecarslamdefault_5 $* &
  racecarslamdefault_6 $* &
  racecarslamdefault_7 $* &
  racecarslamdefault_8 $* &
}

# racecarall
#
# racecarall --focalscale 1.005
# racecarall --focalscale 0.995
# racecarall --focalscale 1.01
# racecarall --focalscale 0.99
#
# racecarall --cxoffset -0.25
# racecarall --cxoffset 0.25
# racecarall --cxoffset -0.5
# racecarall --cxoffset 0.5
# racecarall --cyoffset -0.5
# racecarall --cyoffset 0.5






racecarslampyodo_1() {
    julia -O 3 -p 2 apriltag_and_zed_slam_pynn.jl --folder_name "labrun1" $*
}
racecarslampyodo_2() {
    julia -O 3 -p 2 apriltag_and_zed_slam_pynn.jl --folder_name "labrun2" $*
}
racecarslampyodo_3() {
    julia -O 3 -p 2 apriltag_and_zed_slam_pynn.jl --folder_name "labrun3" $*
}
racecarslampyodo_4() {
    julia -O 3 -p 2 apriltag_and_zed_slam_pynn.jl --folder_name "labrun4" $*
}
racecarslampyodo_5() {
    julia -O 3 -p 2 apriltag_and_zed_slam_pynn.jl --folder_name "labrun5" $*
}
racecarslampyodo_6() {
    julia -O 3 -p 2 apriltag_and_zed_slam_pynn.jl --folder_name "labrun6" $*
}
racecarslampyodo_7() {
    julia -O 3 -p 2 apriltag_and_zed_slam_pynn.jl --folder_name "labrun7" $*
}
racecarslampyodo_8() {
    julia -O 3 -p 2 apriltag_and_zed_slam_pynn.jl --folder_name "labrun8" $*
}


racecarall() {
  racecarslampyodo_1 $* &
  racecarslampyodo_2 $* &
  racecarslampyodo_3 $* &
  racecarslampyodo_4 $* &
  racecarslampyodo_5 $* &
  racecarslampyodo_6 $* &
  racecarslampyodo_7 $* &
  racecarslampyodo_8 $* &
}
