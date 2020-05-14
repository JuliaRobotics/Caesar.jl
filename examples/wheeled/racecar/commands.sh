#!/bin/bash

## requires
# /tmp/caesar/ points to desired results directory also containing the racecar.log file
# $HOME/data to point to folder containing racecar/labrun*

export CAESAR_EX_DIR=$HOME/.julia/dev/Caesar/examples/wheeled/racecar/

# julia102 -O 3 -p 4 apriltag_and_zed_slam.jl --folder_name "labrun8" --failsafe
# julia102 -O 3 -p 4 apriltag_and_zed_slam.jl --folder_name "labrun8" --failsafe --focalscale 1.005
# julia102 -O 3 -p 4 apriltag_and_zed_slam.jl --folder_name "labrun8" --failsafe --focalscale 0.995
# julia102 -O 3 -p 6 apriltag_and_zed_slam.jl --folder_name "labrun8" --failsafe > out8



racecarslamdefault_1() {
    julia -O 3 -p 8 $CAESAR_EX_DIR/apriltag_and_zed_slam.jl --folder_name "labrun1" $*
}
racecarslamdefault_2() {
    julia -O 3 -p 8 $CAESAR_EX_DIR/apriltag_and_zed_slam.jl --folder_name "labrun2" $*
}
racecarslamdefault_3() {
    julia -O 3 -p 8 $CAESAR_EX_DIR/apriltag_and_zed_slam.jl --folder_name "labrun3" $*
}
racecarslamdefault_4() {
    julia -O 3 -p 8 $CAESAR_EX_DIR/apriltag_and_zed_slam.jl --folder_name "labrun4" $*
}
racecarslamdefault_5() {
    julia -O 3 -p 8 $CAESAR_EX_DIR/apriltag_and_zed_slam.jl --folder_name "labrun5" $*
}
racecarslamdefault_6() {
    julia -O 3 -p 8 $CAESAR_EX_DIR/apriltag_and_zed_slam.jl --folder_name "labrun6" $*
}
racecarslamdefault_7() {
    julia -O 3 -p 8 $CAESAR_EX_DIR/apriltag_and_zed_slam.jl --folder_name "labrun7" $*
}
racecarslamdefault_8() {
    julia -O 3 -p 8 $CAESAR_EX_DIR/apriltag_and_zed_slam.jl --folder_name "labrun8" $*
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
    julia -O 3 -p 2 $CAESAR_EX_DIR/apriltag_and_zed_slam_pynn.jl --folder_name "labrun1" $*
}
racecarslampyodo_2() {
    julia -O 3 -p 2 $CAESAR_EX_DIR/apriltag_and_zed_slam_pynn.jl --folder_name "labrun2" $*
}
racecarslampyodo_3() {
    julia -O 3 -p 2 $CAESAR_EX_DIR/apriltag_and_zed_slam_pynn.jl --folder_name "labrun3" $*
}
racecarslampyodo_4() {
    julia -O 3 -p 2 $CAESAR_EX_DIR/apriltag_and_zed_slam_pynn.jl --folder_name "labrun4" $*
}
racecarslampyodo_5() {
    julia -O 3 -p 2 $CAESAR_EX_DIR/apriltag_and_zed_slam_pynn.jl --folder_name "labrun5" $*
}
racecarslampyodo_6() {
    julia -O 3 -p 2 $CAESAR_EX_DIR/apriltag_and_zed_slam_pynn.jl --folder_name "labrun6" $*
}
racecarslampyodo_7() {
    julia -O 3 -p 2 $CAESAR_EX_DIR/apriltag_and_zed_slam_pynn.jl --folder_name "labrun7" $*
}
racecarslampyodo_8() {
    julia -O 3 -p 2 $CAESAR_EX_DIR/apriltag_and_zed_slam_pynn.jl --folder_name "labrun8" $*
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
    julia -O 3 $CAESAR_EX_DIR/apriltag_and_zed_slam_flux.jl --folder_name "labrun1" $*
}
racecarslamflux_2() {
    julia -O 3 $CAESAR_EX_DIR/apriltag_and_zed_slam_flux.jl --folder_name "labrun2" $*
}
racecarslamflux_3() {
    julia -O 3 $CAESAR_EX_DIR/apriltag_and_zed_slam_flux.jl --folder_name "labrun3" $*
}
racecarslamflux_4() {
    julia -O 3 $CAESAR_EX_DIR/apriltag_and_zed_slam_flux.jl --folder_name "labrun4" $*
}
racecarslamflux_5() {
    julia -O 3 $CAESAR_EX_DIR/apriltag_and_zed_slam_flux.jl --folder_name "labrun5" $*
}
racecarslamflux_6() {
    julia -O 3 $CAESAR_EX_DIR/apriltag_and_zed_slam_flux.jl --folder_name "labrun6" $*
}
racecarslamflux_7() {
    julia -O 3 $CAESAR_EX_DIR/apriltag_and_zed_slam_flux.jl --folder_name "labrun7" $*
}
racecarslamflux_8() {
    julia -O 3 $CAESAR_EX_DIR/apriltag_and_zed_slam_flux.jl --folder_name "labrun8" $*
}


racecarslamfluxall() {
  sleep 01; racecarslamflux_1 $* &
  sleep 60; racecarslamflux_2 $* &
  sleep 120; racecarslamflux_3 $* &
  sleep 180; racecarslamflux_4 $* &
  sleep 240; racecarslamflux_5 $* &
  sleep 300; racecarslamflux_6 $* &
  sleep 360; racecarslamflux_7 $* &
  sleep 420; racecarslamflux_8 $* &
}

# get user slam procs
getjuliaprocs() {
  ps -U $USER | grep "julia" | grep -v grep | awk '{print $1}'
}

killjuliaprocs() {
  getjuliaprocs | xargs kill -9
}

last8log() {
  # tail -n8 /tmp/caesar/racecar.log | awk '{print $1}' | sed 's/,//g'
  tail -n8 /tmp/caesar/results.log | awk '{print $1}' | sed 's/,//g'
}

racecarresultslabrun() {
  # julia -O3 -e "splitpath(\"`cat /tmp/caesar/$ln/readme.txt | head -n2 | tail -n1`\")[end] |> println"
  julia -O3 -e "using JSON; JSON.parsefile(\"/tmp/caesar/$1/args.json\")[\"folder_name\"] |> println"
}

copylatesttoconductor() {
  last8log > /tmp/caesar/last8
  while read ln; do
    echo "Copy to conductor $ln"
    # get labrun
    WHICHRES=`racecarresultslabrun $ln`
    # racecarresultslabrun $ln > /tmp/caesar/whichresults
    # WHICHRES=`cat /tmp/caesar/whichresults`

    echo $ln > /tmp/caesar/conductor/solves/$WHICHRES.aux
    # cp -f /tmp/caesar/$ln/results/results.csv /tmp/caesar/conductor/solves/results_$WHICHRES.csv
    # cp -f /tmp/caesar/$ln/results/results.csv /home/singhk/data/racecar/$WHICHRES/results_$WHICHRES.csv
    cp -f /tmp/caesar/$ln/${WHICHRES}_results.json /tmp/caesar/conductor/solves/
    # cp -f /tmp/caesar/$ln/${WHICHRES}_results.json /home/singhk/data/racecar/$WHICHRES/
    #also copy the latest image
    cp -f /tmp/caesar/$ln/*_resolve.pdf /tmp/caesar/conductor/solves/${WHICHRES}_resolve.pdf
    # LSTIMG=`ls -t /tmp/caesar/$ln/images | head -n20 | grep -v "hist" | head -n1`
    # cp -f /tmp/caesar/$ln/images/$LSTIMG /tmp/caesar/conductor/solves/img_${WHICHRES}_${LSTIMG}
  done < /tmp/caesar/last8
}



getFrac() {
  A=`julia -O3 -e "(11-$1)/10 |> print"`
  echo $A
}

# racecarslamrosfluxall --localprocs 2 --remoteprocs 4 --imshow --naive_frac 0.7
# racecarslamrosfluxall --localprocs 8 --remoteprocs 0 --naive_frac `getFrac $i`

racecarslamros() {
    JULIA_NUM_THREADS=4 julia -O 3 $CAESAR_EX_DIR/ros/CarFeedMono.jl $* --batch_resolve --vis2d
}

racecarslamrosall() {
  sleep 00; racecarslamros --folder_name "labrun7" $* &
  sleep 60; racecarslamros --folder_name "labrun6" $* &
  sleep 60; racecarslamros --folder_name "labrun8" $*
  sleep 00; racecarslamros --folder_name "labrun4" $* &
  sleep 60; racecarslamros --folder_name "labrun2" $* &
  sleep 60; racecarslamros --folder_name "labrun3" $*
  sleep 00; racecarslamros --folder_name "labrun5" $* &
  sleep 60; racecarslamros --folder_name "labrun1" $*
}

racecarslamrosflux() {
    JULIA_NUM_THREADS=4 julia -O 3 $CAESAR_EX_DIR/ros/CarFeedMonoFlux.jl $* --batch_resolve --vis2d --savedfg
}

racecarslamrosfluxall() {
  sleep 00; racecarslamrosflux --folder_name "labrun7" $* &
  sleep 60; racecarslamrosflux --folder_name "labrun6" $* &
  sleep 60; racecarslamrosflux --folder_name "labrun8" $*
  sleep 00; racecarslamrosflux --folder_name "labrun4" $* &
  sleep 60; racecarslamrosflux --folder_name "labrun2" $* &
  sleep 60; racecarslamrosflux --folder_name "labrun3" $*
  sleep 00; racecarslamrosflux --folder_name "labrun5" $* &
  sleep 60; racecarslamrosflux --folder_name "labrun1" $*
}

racecarslamrosfluxALL() {
  sleep 00; racecarslamrosflux --folder_name "labrun7" $* &
  sleep 80; racecarslamrosflux --folder_name "labrun6" $* &
  sleep 80; racecarslamrosflux --folder_name "labrun8" $* &
  sleep 80; racecarslamrosflux --folder_name "labrun4" $* &
  sleep 80; racecarslamrosflux --folder_name "labrun2" $* &
  sleep 80; racecarslamrosflux --folder_name "labrun3" $* &
  sleep 80; racecarslamrosflux --folder_name "labrun5" $* &
  sleep 80; racecarslamrosflux --folder_name "labrun1" $*
}



## analysis runs

# racecarslamrosfluxall --localprocs 4 --remoteprocs 7 --imshow --naive_frac 0.9
# JULIA_NUM_THREADS=2 racecarslamrosfluxALL --localprocs 0 --remoteproc 10 --imshow --naive_frac 1.0 --pose_trigger_distance 0.1
# racecarslamrosflux --folder_name "labrun7" --localprocs 0 --remoteproc 15 --imshow --naive_frac 1.0 --pose_trigger_distance 0.2 &

# racecarslamrosflux_analysis1() {
#   racecarslamrosfluxall --localprocs 2 --remoteprocs 4 --imshow --naive_frac 1.0
#   racecarslamrosfluxall --localprocs 2 --remoteprocs 4 --imshow --naive_frac 0.9
#   racecarslamrosfluxall --localprocs 2 --remoteprocs 4 --imshow --naive_frac 0.8
#   racecarslamrosfluxall --localprocs 2 --remoteprocs 4 --imshow --naive_frac 0.7
#   racecarslamrosfluxall --localprocs 2 --remoteprocs 4 --imshow --naive_frac 0.6
#   racecarslamrosfluxall --localprocs 2 --remoteprocs 4 --imshow --naive_frac 0.5
# }

delaybashracecarjulias() {
  while [ 0 -lt `getjuliaprocs | wc -l` ]; do
    echo "waiting for julia procs to finish, /tmp/juliaprocs="
    echo `getjuliaprocs`
    getjuliaprocs > /tmp/caesar/juliaprocs
    sleep 30;
  done
}



racecarpynnconductor() {
  racecarslamrosfluxALL $*
  # racecarslampynnall
  sleep 60

  delaybashracecarjulias

  copylatesttoconductor

}


  # racecarslamrosfluxall --localprocs 2 --remoteprocs 4 --imshow --naive_frac 1.0;  racecarslamrosfluxall --localprocs 2 --remoteprocs 4 --imshow --naive_frac 0.9;  racecarslamrosfluxall --localprocs 2 --remoteprocs 4 --imshow --naive_frac 0.8;  racecarslamrosfluxall --localprocs 2 --remoteprocs 4 --imshow --naive_frac 0.7



racecartrainflux() {
  julia -O 3 $CAESAR_EX_DIR/FluxModelsTraining.jl $*
}

racecartrainfluxSINGLE() {
  racecartrainflux --numFGDatasets 1 $*
}

racecartrainfluxBASIC() {
  racecartrainfluxSINGLE --epochsFlux 1 --fluxGenerations 2 --rndSkip 10 $*
}

racecartrainfluxBASIC10cm() {
  racecartrainfluxBASIC --fgpathsflux "/tmp/caesar/conductor/fluxtrain/distance10cm_0.txt" $*
}

# JULIA_NUM_THREADS=10 racecartrainflux --numFGDatasets 6 --epochsFlux 5 --fluxGenerations 5 --rndSkip 10 --fgpathsflux "/tmp/caesar/conductor/fluxtrain/distance10cm_0.txt" --trainingNumOffset 10000

racecartrainflux10times() {
  racecartrainflux
  racecartrainflux
  racecartrainflux
  racecartrainflux
  racecartrainflux
  racecartrainflux
  racecartrainflux
  racecartrainflux
  racecartrainflux
  racecartrainflux
}




# racecarslamrosfluxall --localprocs 6 --remoteprocs 0 --imshow --naive_frac 1.0 --pose_trigger_distance 0.2



##
