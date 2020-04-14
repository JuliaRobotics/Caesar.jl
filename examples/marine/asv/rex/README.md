

* `RExFeed` - subscribes to the ROS topics and builds a DFG object
* `RExRadarProcessor` - loads the DFG object, assembles full 360deg scans, and pushes them back to the DFG.
* `ScanMatcher` - loads the DFG object, computes odometry estimates from sequential radar scans, and adds them to the factor graph
