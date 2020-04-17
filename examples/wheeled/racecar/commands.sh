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


racecarslampynnall() {
  sleep 01; racecarslampyodo_1 $* &
  sleep 30; racecarslampyodo_2 $* &
  sleep 60; racecarslampyodo_3 $* &
  sleep 90; racecarslampyodo_4 $* &
  sleep 120; racecarslampyodo_5 $* &
  sleep 150; racecarslampyodo_6 $* &
  sleep 180; racecarslampyodo_7 $* &
  sleep 210; racecarslampyodo_8 $* &
}



racecarslamflux_1() {
    julia -O 3 -p 5 apriltag_and_zed_slam_flux.jl --folder_name "labrun1" $*
}
racecarslamflux_2() {
    julia -O 3 -p 5 apriltag_and_zed_slam_flux.jl --folder_name "labrun2" $*
}
racecarslamflux_3() {
    julia -O 3 -p 5 apriltag_and_zed_slam_flux.jl --folder_name "labrun3" $*
}
racecarslamflux_4() {
    julia -O 3 -p 5 apriltag_and_zed_slam_flux.jl --folder_name "labrun4" $*
}
racecarslamflux_5() {
    julia -O 3 -p 5 apriltag_and_zed_slam_flux.jl --folder_name "labrun5" $*
}
racecarslamflux_6() {
    julia -O 3 -p 5 apriltag_and_zed_slam_flux.jl --folder_name "labrun6" $*
}
racecarslamflux_7() {
    julia -O 3 -p 5 apriltag_and_zed_slam_flux.jl --folder_name "labrun7" $*
}
racecarslamflux_8() {
    julia -O 3 -p 5 apriltag_and_zed_slam_flux.jl --folder_name "labrun8" $*
}


racecarslamfluxall() {
  sleep 01; racecarslamflux_1 $* &
  sleep 30; racecarslamflux_2 $* &
  sleep 60; racecarslamflux_3 $* &
  sleep 90; racecarslamflux_4 $* &
  sleep 120; racecarslamflux_5 $* &
  sleep 150; racecarslamflux_6 $* &
  sleep 180; racecarslamflux_7 $* &
  sleep 210; racecarslamflux_8 $* &
}

# get user slam procs
getjuliaprocs() {
  ps | grep julia | awk '{print $1}' > /tmp/caesar/juliaprocs
}

killjuliaprocs() {
  getjuliaprocs
  cat /tmp/caesar/juliaprocs | xargs kill -9
}

last8log() {
  tail -n8 /tmp/caesar/racecar.log | awk '{print $1}' | sed 's/,//g'
}

copylatesttoconductor() {
  last8log > /tmp/caesar/last8
  while read ln; do
    echo "Copy to conductor $ln"
    cp -r /tmp/caesar/$ln /tmp/caesar/conductor/solves/
  done < /tmp/caesar/last8
}

racecarpynnconductor() {
  racecarslamfluxall
  # racecarslampynnall
  sleep 60
  getjuliaprocs

  while [ 0 -lt `wc -l /tmp/caesar/juliaprocs | awk '{print $1}'` ]; do
    echo "waiting for julia procs to finish, /tmp/juliaprocs="
    cat /tmp/caesar/juliaprocs
    sleep 30;
  done

  copylatesttoconductor

}
