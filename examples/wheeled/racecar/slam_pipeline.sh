#!/bin/bash

JULIACMD=julia101

slam_labrun5() {
  JULIACMD apriltag_and_zed_slam.jl $*
}



visualize_results() {
  # will do all plotting from here
}


#
