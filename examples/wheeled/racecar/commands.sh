#!/bin/bash

# julia102 -O 3 -p 4 apriltag_and_zed_slam.jl --folder_name "labrun8" --failsafe
# julia102 -O 3 -p 4 apriltag_and_zed_slam.jl --folder_name "labrun8" --failsafe --focalscale 1.005
# julia102 -O 3 -p 4 apriltag_and_zed_slam.jl --folder_name "labrun8" --failsafe --focalscale 0.995
# julia102 -O 3 -p 6 apriltag_and_zed_slam.jl --folder_name "labrun8" --failsafe > out8



racecarslamdefault_1() {
    julia102 -O 3 -p 4 apriltag_and_zed_slam.jl --folder_name "labrun1" --failsafe $*
}
racecarslamdefault_2() {
    julia102 -O 3 -p 4 apriltag_and_zed_slam.jl --folder_name "labrun2" --failsafe $*
}
racecarslamdefault_3() {
    julia102 -O 3 -p 4 apriltag_and_zed_slam.jl --folder_name "labrun3" --failsafe $*
}
racecarslamdefault_4() {
    julia102 -O 3 -p 4 apriltag_and_zed_slam.jl --folder_name "labrun4" --failsafe $*
}
racecarslamdefault_5() {
    julia102 -O 3 -p 4 apriltag_and_zed_slam.jl --folder_name "labrun5" --failsafe $*
}
racecarslamdefault_6() {
    julia102 -O 3 -p 4 apriltag_and_zed_slam.jl --folder_name "labrun6" --failsafe $*
}
racecarslamdefault_7() {
    julia102 -O 3 -p 4 apriltag_and_zed_slam.jl --folder_name "labrun7" --failsafe $*
}
racecarslamdefault_8() {
    julia102 -O 3 -p 4 apriltag_and_zed_slam.jl --folder_name "labrun8" --failsafe $*
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

racecarsweepfocal() {
  #racecarall --focalscale 1.0
  racecarall --focalscale 0.99
  racecarall --focalscale 0.98
  sleep 1000
  racecarall --focalscale 0.97
  sleep 1000
  racecarall --focalscale 0.96

  #racecarall --focalscale 0.95

  sleep 1000
  racecarall --focalscale 0.94
  sleep 1000
  racecarall --focalscale 0.93

  #racecarall --focalscale 0.90
  #racecarall --focalscale 0.85
  #racecarall --focalscale 0.80
  #racecarall --focalscale 0.75
  #racecarall --focalscale 0.70
  #racecarall --focalscale 0.65
  #racecarall --focalscale 0.60
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
