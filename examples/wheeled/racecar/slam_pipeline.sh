#!/bin/bash

JULIACMD=julia101

slam_labruns() {
  for runNum in {1..8}
  do
    for fx in {330..350..4}
    do
      for fy in {330..350..4}
      do
        for cx in {315..340..5}
        do
          for cy in {185..220..5}
          do
            JULIACMD apriltag_and_zed_slam.jl --folder_name "labrun$runNum" --cx cx --cy cy --fx fx --fy fy
}



visualize_results() {
  # will do all plotting from here
}


slam_labruns()
