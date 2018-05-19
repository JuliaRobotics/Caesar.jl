# Caesar.jl Automated Camera Calibration using AprilTags

This tutorial describes a mechanism to calibration camera calibration parameters using factor graphs and [AprilTags.jl](http://www.github.com/JuliaRobotics/AprilTags.jl).  The intention of the method is to leverage a technique from simultaneous localization and mapping (SLAM) that would allow estimation of intrinsic and extrinsic calibration parameters without requiring an accurate or large poster pattern printout, or an highly involved calibration procedure.

## Print-and-Place a few AprilTags

**TODO**: Follow [this link](http://www.github.com/JuliaRobotics/AprilTags.jl/standardtags) and print out the five tags contained in the pdf file.  Randomly these tags at different distances and not all on the same surface so that you can move the camera around a in a few good viewing positions later on.

The code described in this tutorial is uses a few hand captured images from the camera, but can also be used to directly use the live camera feed for a more unified calibration process.

## Factor Graph Strategy

Factor graphs are a language with which an inference problem can be defined, which is used to capture several camera based measurements of the semi-known AprilTags.  The [Caesar.jl](http://www.github.com/dehann/Caesar.jl) and associated packages implement the [multimodal iSAM](https://darchive.mblwhoilibrary.org/bitstream/handle/1912/9305/Fourie_thesis.pdf?sequence=1) inference algorithm that is used to solve the given factor graph algorithm.  

### Keyframes and Poses

By using a few keyframe images that are separated by one or two seconds each as the camera is slowly moved around in the scene, a chain of corresponding camera poses are created.  The mm-iSAM estimation process will use the combination of the pose chain, as well as the AprilTag detections in each image to construct an inference problem that finds the intrinsic camera calibration parameters (camera focal length and optical center).  The poses and tag positions is graphical illustrated in a factor graph depiction below.

INSERT FG IMAGE HERE
