#!/usr/bin/env python

# Author: Nick R. Rypkema (rypkema@mit.edu)
# License: MIT

''' Dependencies '''
import time
import struct
import os
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from neo4j_beamform import neo4j_beamform
from neo4j_interact_segments import neo4j_interact
from neo4j_interact_segments import Noise1D, Noise2D, Noise3D
from neo4j_interact_segments import priorFactorPoint2, priorFactorPose2, betweenFactorPose2, rangeMeasurementFactorPose2Point2, bearingRangeMeasurementFactorPose2Point2, rangeNullHypothesisFactorPose2Point2
from mongo_interact import mongo_interact

class neo4j_slam(object):
    ''' Interface to automatically create odometry nodes and measurement factors to push to Neo4j database '''
    def __init__(self, authfile, session_name, mongo_authfile, mongo_collection_name, num_particles, equator_segment_angle, azimuth_bins, heading_offset, speed_scaling, source_latitude, source_longitude, slam_delta_t, use_lbl=False, use_lbl_prior=False):
        ''' user-set parameters '''
        self.source_lon = source_longitude
        self.source_lat = source_latitude
        self.num_particles = num_particles
        self.equator_segment_angle = equator_segment_angle
        self.azimuth_bins = azimuth_bins
        self.yaw_offset = heading_offset
        self.speed_scale = speed_scaling
        self.slam_delta_t = slam_delta_t

        ''' Neo4j SLAM parameters '''
        self.prior_noise = Noise3D(2.0, 2.0, 5*math.pi/180.0)
        self.source_prior_noise = Noise2D(0.1, 0.1)
        self.odometry_noise = Noise3D(0.2, 0.05, 1*math.pi/180.0)
        self.lbl_range_noise = Noise1D(2.5)
        self.lbl_null_probability = 0.5
        self.use_lbl = use_lbl
        self.segment_val = 1

        ''' temporary parameters '''
        self.isam_t_accum = None
        self.isam_theta_prev = None
        self.isam_odo_x_accum = None
        self.isam_odo_y_accum = None
        self.ddrkn_x = None
        self.ddrkn_y = None

        ''' initialize Neo4j beamformer '''
        self.neo4j_bf = neo4j_beamform(self.num_particles, self.equator_segment_angle, self.azimuth_bins)

        ''' initialize Neo4j interface '''
        self.neo4j_iface = neo4j_interact(authfile, session_name)
        self.source_id = self.neo4j_iface.add_landmark(0, 0, priorFactorPoint2(0, 0, self.source_prior_noise))

        ''' initialize mongo interface '''
        self.mongo_iface = mongo_interact(mongo_authfile, session_name, mongo_collection_name)

        ''' LBL usage '''
        if self.use_lbl:
            if use_lbl_prior:
                self.lbl1_id = self.neo4j_iface.add_landmark(52.808237599621172, 23.882253538539306, priorFactorPoint2(52.808237599621172, 23.882253538539306, self.source_prior_noise))
                self.lbl2_id = self.neo4j_iface.add_landmark(-72.498048500886142, -32.990833958689464, priorFactorPoint2(-72.498048500886142, -32.990833958689464, self.source_prior_noise))
            else:
                self.lbl1_id = self.neo4j_iface.add_landmark(52.808237599621172, 23.882253538539306)
                self.lbl2_id = self.neo4j_iface.add_landmark(-72.498048500886142, -32.990833958689464)

    def initialize(self, gps_lat, gps_lon, nav_depth, nav_yaw):
        gps_x, gps_y = self.xy_meters(self.source_lat, self.source_lon, gps_lat, gps_lon)
        yaw = nav_yaw + self.yaw_offset
        yaw = self.heading_to_pi(math.radians(yaw))
        self.latest_pose_id = self.neo4j_iface.add_pose(self.segment_val, gps_x, gps_y, yaw, priorFactorPose2(gps_x, gps_y, yaw, self.prior_noise), None)
        self.ddrkn_x = gps_x
        self.ddrkn_y = gps_y

    def run(self, dt, pitch, roll, yaw, depth, speed, cbf_data=None, gps_lat=None, gps_lon=None, gps_accuracy=None, lbl1_range=None, lbl2_range=None):
        # tic = time.time()

        r_vals = None
        az_vals = None
        if cbf_data is not None:
            r_vals, az_vals, r_max, p_max = self.neo4j_bf.run(pitch, roll, depth, cbf_data)

        yaw = yaw + self.yaw_offset
        theta = self.heading_to_pi(math.radians(yaw))
        speed = speed * self.speed_scale

        if self.isam_t_accum is None:
            self.isam_t_accum = 0.0
        if self.isam_theta_prev is None:
            self.isam_theta_prev = theta
        if self.isam_odo_x_accum is None:
            self.isam_odo_x_accum = 0.0
        if self.isam_odo_y_accum is None:
            self.isam_odo_y_accum = 0.0

        self.propagate_dead_reckon(yaw, speed, dt)
        dr_azim = math.atan2(self.ddrkn_y, self.ddrkn_x) - theta - math.pi
        if dr_azim < 0:
            dr_azim += 2*math.pi
        dr_range = math.hypot(self.ddrkn_x, self.ddrkn_y)

        update = False
        self.isam_t_accum += dt
        isam_yaw = theta - self.isam_theta_prev
        self.propagate_odometry(isam_yaw, speed, dt)
        if self.isam_t_accum >= self.slam_delta_t:
            odometry = betweenFactorPose2(self.isam_odo_x_accum, self.isam_odo_y_accum, isam_yaw, self.odometry_noise)
            self.latest_pose_id = self.neo4j_iface.add_pose(self.segment_val, None, None, None, None, odometry)
            self.isam_t_accum = 0.0
            self.isam_theta_prev = theta
            self.isam_odo_x_accum = 0.0
            self.isam_odo_y_accum = 0.0
            update = True

        if (r_vals is not None) and (az_vals is not None):
            neo4j_id = self.neo4j_iface.add_measurement(self.segment_val, bearingRangeMeasurementFactorPose2Point2(self.source_id, [], []))
            mongo_key_az = self.mongo_iface.insert_array(az_vals)
            mongo_key_r = self.mongo_iface.insert_array(r_vals)
            self.neo4j_iface.add_mongo_key(neo4j_id, 'bearing', str(mongo_key_az))
            self.neo4j_iface.add_mongo_key(neo4j_id, 'range', str(mongo_key_r))
        elif (r_vals is not None):
            neo4j_id = self.neo4j_iface.add_measurement(self.segment_val, rangeMeasurementFactorPose2Point2(self.source_id, []))
            mongo_key_r = self.mongo_iface.insert_array(r_vals)
            self.neo4j_iface.add_mongo_key(neo4j_id, 'range', str(mongo_key_r))
        if self.use_lbl:
            if lbl1_range is not None:
                self.neo4j_iface.add_measurement(self.segment_val, rangeNullHypothesisFactorPose2Point2(self.lbl1_id, lbl1_range, self.lbl_null_probability, self.lbl_range_noise))
            if lbl2_range is not None:
                self.neo4j_iface.add_measurement(self.segment_val, rangeNullHypothesisFactorPose2Point2(self.lbl2_id, lbl2_range, self.lbl_null_probability, self.lbl_range_noise))

        if (gps_accuracy is not None) and update:
            if gps_accuracy <= 4.0:
                gps_x, gps_y = self.xy_meters(self.source_lat, self.source_lon, gps_lat, gps_lon)
                self.segment_val += 1
                self.latest_pose_id = self.neo4j_iface.add_pose(self.segment_val, gps_x, gps_y, theta, priorFactorPose2(gps_x, gps_y, theta, self.prior_noise), None)
                self.ddrkn_x = gps_x
                self.ddrkn_y = gps_y

        # print 'Neo4j SLAM loop time:', time.time()-tic

        return self.ddrkn_x, self.ddrkn_y, dr_range, dr_azim

    def propagate_odometry(self, yaw, speed_over_ground, delta_t):
        dr = speed_over_ground*delta_t
        self.isam_odo_x_accum += dr*np.cos(yaw)
        self.isam_odo_y_accum += dr*np.sin(yaw)

    def propagate_dead_reckon(self, yaw, speed_over_ground, delta_t):
        dr = speed_over_ground*delta_t
        yaw = self.heading_to_pi(math.radians(yaw))
        self.ddrkn_x += dr*np.cos(yaw)
        self.ddrkn_y += dr*np.sin(yaw)

    def heading_to_pi(self, val):
        ret = val
        ret = 2*np.pi - ret
        ret = ret - 1.5*np.pi
        if ret < -np.pi:
            ret = ret + 2*np.pi
        return ret

    def N(self, phi):
        a = 6378137
        e = 0.08181919084
        return a/(1-(e**2)*(math.sin(math.radians(phi)))**2)**0.5

    def M(self, phi):
        a = 6378137
        e = 0.08181919084
        return (a*(1-e**2))/(1-(e**2)*(math.sin(math.radians(phi)))**2)**1.5

    def xy_meters(self, lat1, lon1, lat2, lon2):
        RN = self.N(lat1)
        RM = self.M(lat1)
        dist_X = RN*math.cos(math.radians(lat1))*(lon2-lon1)*np.pi/180
        dist_Y = RM*(lat2-lat1)*np.pi/180
        return dist_X, dist_Y

    def pi_to_heading(self, val):
        ret = val
        if val < 0:
            ret = ret + 2*np.pi
        ret = ret + 1.5*np.pi
        if ret > 2*np.pi:
            ret = ret - 2*np.pi
        ret = 2*np.pi - ret
        return ret

    def heading_to_pi(self, val):
        ret = val
        ret = 2*np.pi - ret
        ret = ret - 1.5*np.pi
        if ret < -np.pi:
            ret = ret + 2*np.pi
        return ret