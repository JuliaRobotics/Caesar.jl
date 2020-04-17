#!/usr/bin/env python

# Author: Nick R. Rypkema (rypkema@mit.edu)
# License: MIT

''' 
Example usage of SLAMinDB interface
'''

''' Dependencies '''
import time
import struct
import os
import numpy as np
import math
from neo4j_interact import neo4j_interact
from neo4j_interact import Noise1D, Noise2D, Noise3D
from neo4j_interact import priorFactorPoint2, priorFactorPose2, betweenFactorPose2, rangeFactorPose2Point2, bearingRangeFactorPose2Point2

authfile = './neo_authfile.txt'
session_name = 'NEO4J_EXAMPLE'

# initialize the Neo4j SLAMinDB interface
neo4j_iface = neo4j_interact(authfile=authfile, session_name=session_name)

# create noise values for priors, odometry, and measurements 
priorNoise = Noise3D(0.2, 0.2, 0.05)
landmarkPriorNoise = Noise2D(0.002, 0.002)
odometryNoise = Noise3D(0.1, 0.05, 0.01)
rangeNoise = Noise1D(0.25)
bearingRangeNoise = Noise2D(0.1, 0.25)

# add a landmark to our Neo4j graph at (0,0)
lm_id = neo4j_iface.add_landmark(0, 0, priorFactorPoint2(0, 0, landmarkPriorNoise))

# add an initial pose at (1,1,-pi/2) with prior factor at this position
neo4j_iface.add_pose(1, 1, -math.pi/2, priorFactorPose2(1, 1, -math.pi/2, priorNoise), None)

# lets create an odometry factor of (delta_surge,delta_sway,delta_theta)=(2,0,-pi/2) to continuously drive in a square of size 2x2 with some random Gaussian noise
odometry = betweenFactorPose2(2+np.random.normal(0, 0.1), 0+np.random.normal(0, 0.05), -math.pi/2+np.random.normal(0, 0.01), odometryNoise)

# add a second pose using our odometry factor, with a bearing/range measurement to the landmark with some random Gaussian noise
neo4j_iface.add_pose(None, None, None, None, odometry)
neo4j_iface.add_measurement(bearingRangeFactorPose2Point2(lm_id, -math.pi/4+np.random.normal(0, 0.1), math.sqrt(2.0)+np.random.normal(0, 0.25), bearingRangeNoise))

# add a third pose using our odometry factor with some new random Gaussian noise
odometry = betweenFactorPose2(2+np.random.normal(0, 0.1), 0+np.random.normal(0, 0.05), -math.pi/2+np.random.normal(0, 0.01), odometryNoise)
neo4j_iface.add_pose(None, None, None, None, odometry)

# add a fourth pose using our odometry factor, with a range measurement to the landmark with some random Gaussian noise
neo4j_iface.add_pose(None, None, None, None, odometry)
neo4j_iface.add_measurement(rangeFactorPose2Point2(lm_id, math.sqrt(2.0)+np.random.normal(0, 0.25), rangeNoise))

# add a fifth pose using our odometry factor with some new random Gaussian noise
odometry = betweenFactorPose2(2+np.random.normal(0, 0.1), 0+np.random.normal(0, 0.05), -math.pi/2+np.random.normal(0, 0.01), odometryNoise)
neo4j_iface.add_pose(None, None, None, None, odometry)

# add another prior pose (e.g. we get a GPS fix) at (1,-1,-pi) with prior factor at this position
neo4j_iface.add_pose(1, -1, -math.pi, priorFactorPose2(1, -1, -math.pi, priorNoise), None)

# add a seventh pose using our odometry factor, with a bearing/range measurement to the landmark with some random Gaussian noise
odometry = betweenFactorPose2(2+np.random.normal(0, 0.1), 0+np.random.normal(0, 0.05), -math.pi/2+np.random.normal(0, 0.01), odometryNoise)
neo4j_iface.add_pose(None, None, None, None, odometry)
neo4j_iface.add_measurement(bearingRangeFactorPose2Point2(lm_id, -math.pi/4+np.random.normal(0, 0.1), math.sqrt(2.0)+np.random.normal(0, 0.25), bearingRangeNoise))

# add a eighth pose using our odometry factor, with a range measurement to the landmark with some random Gaussian noise
neo4j_iface.add_pose(None, None, None, None, odometry)
neo4j_iface.add_measurement(rangeFactorPose2Point2(lm_id, math.sqrt(2.0)+np.random.normal(0, 0.25), rangeNoise))

time.sleep(5)