#!/usr/bin/env python

# Author: Nick R. Rypkema (rypkema@mit.edu)
# License: MIT

'''
Simple functions to interface with SLAMinDB
'''

'''
WIP: x, y parameters passed into add_* are used in GTSAM to initialize the optimization solution - in SLAMinDB, initialization
is performed using prior factors (I think), and so these parameters remain unused; we keep them in case the interface changes in the future
'''

''' Dependencies '''
from neo4j.v1 import GraphDatabase, basic_auth
import json

# noise classes to encapsulate noise types for Neo4j database
class Noise1D(object):
    def __init__(self, x):
        self.x = x

class Noise2D(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Noise3D(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

# factor classes to encapsulate factor types for Neo4j database
class priorFactorPoint2(object):
    ''' prior factor for Point2 type (e.g. landmarks) '''
    def __init__(self, x, y, noise):
        if type(noise) is not Noise2D:
            raise TypeError("noise must be of type Noise2D")
        self.x = x
        self.y = y
        self.noise = noise

class priorFactorPose2(object):
    ''' prior factor for Pose2 type '''
    def __init__(self, x, y, theta, noise):
        if type(noise) is not Noise3D:
            raise TypeError("noise must be of type Noise3D")
        self.x = x
        self.y = y
        self.theta = theta
        self.noise = noise

class betweenFactorPose2(object):
    ''' factor linking two pose2 types (e.g. odometry) '''
    def __init__(self, delta_surge, delta_sway, delta_theta, noise):
        if type(noise) is not Noise3D:
            raise TypeError("noise must be of type Noise3D")
        self.delta_surge = delta_surge
        self.delta_sway = delta_sway
        self.delta_theta = delta_theta
        self.noise = noise

class rangeMeasurementFactorPose2Point2(object):
    ''' factor containing a range distribution as a list of range values '''
    def __init__(self, landmark_id, range_val):
        self.landmark_id = landmark_id
        self.range_val = range_val

class bearingRangeMeasurementFactorPose2Point2(object):
    ''' factor containing a range and bearing distribution as lists of range and bearing values '''
    def __init__(self, landmark_id, bearing_val, range_val):
        self.landmark_id = landmark_id
        self.bearing_val = bearing_val
        self.range_val = range_val

class rangeFactorPose2Point2(object):
    ''' unimodal range factor '''
    def __init__(self, landmark_id, range_val, noise):
        if type(noise) is not Noise1D:
            raise TypeError("noise must be of type Noise1D")
        self.landmark_id = landmark_id
        self.range_val = range_val
        self.noise = noise

class bearingRangeFactorPose2Point2(object):
    ''' unimodal bearing and range factor '''
    def __init__(self, landmark_id, bearing_val, range_val, noise):
        if type(noise) is not Noise2D:
            raise TypeError("noise must be of type Noise2D")
        self.landmark_id = landmark_id
        self.bearing_val = bearing_val
        self.range_val = range_val
        self.noise = noise

class neo4j_interact(object):
    ''' Non-parametric SLAMinDB Neo4j interface '''
    def __init__(self, authfile, session_name):
        self._authfile = authfile
        self._session_name = session_name
        self._num_variables = 0
        self._num_landmarks = 0
        self._uid_landmarks = {}
        self._pose_to_odometry_or_prior = {}
        self._pose_to_measurements = {}
        self._landmark_to_prior = {}
        self._pose_ids = []
        self._max_factor_id = 0

        # initialize Neo4j session
        self.username, self.password, self.DB_address = open(self._authfile).read().splitlines()
        print self.username, self.password, self.DB_address
        self.driver = GraphDatabase.driver(self.DB_address, auth=basic_auth(self.username, self.password))
        self.session = self.driver.session()
        self.session.run("MATCH (n:" + self._session_name + ") DETACH DELETE n")

    # adds landmark to DB, with Point2 prior if given
    def add_landmark(self, x, y, prior_factor=None, uid=None):
        if prior_factor is not None and type(prior_factor) is not priorFactorPoint2:
            raise TypeError("prior_factor must be of type priorFactorPoint2")
        l_id = self._num_landmarks
        if uid is None:
            l_id = uid
        else:
            self._num_landmarks += 1
        l_neo4j = json.dumps({'tag_id': l_id, 'uid': l_id, 't': 'L'})
        self._landmark_to_prior[l_id] = []
        if prior_factor is not None:
            l_prior_factor_neo4j = json.dumps({'meas': str(prior_factor.x) + ' ' + str(prior_factor.y) + ' ' + str(prior_factor.noise.x) + ' 0 ' + str(prior_factor.noise.y),
                                                'lklh': 'PTPR2 G 2',
                                                'btwn': str(l_id),
                                                't': 'F'})
            self._landmark_to_prior[l_id].append(self._max_factor_id)
            self._max_factor_id += 1
            self.session.run("MERGE (l:LANDMARK:"+self._session_name+":NEWDATA { frtend: {landmark_info} }) "
                             "MERGE (f:FACTOR:"+self._session_name+":NEWDATA { frtend: {prior_factor_info} }) "
                             "MERGE (l)-[:DEPENDENCE]-(f)",
                            {"landmark_info": l_neo4j,
                             "prior_factor_info": l_prior_factor_neo4j})
        else:
            self.session.run("MERGE (l:LANDMARK:"+self._session_name+":NEWDATA { frtend: {landmark_info} }) ",
                            {"landmark_info": l_neo4j})
        return l_id

    # adds pose to DB, with Pose2 prior if given, or using an odometry factor to the latest pose if given
    def add_pose(self, x=None, y=None, theta=None, prior_factor=None, odometry_factor=None):
        if x is not None and y is not None and theta is not None and prior_factor is None:
            sel = 'no_factor'
        elif x is not None and y is not None and theta is not None and prior_factor is not None:
            sel = 'prior'
        elif odometry_factor is not None:
            sel = 'odometry'
        else:
            raise ValueError("three choices:\n  - x, y, theta not None\n  - x, y, theta, prior_factor not None\n  - odometry_factor not None")

        if sel == 'no_factor':
            p_id, res = self._add_no_factor_pose(x, y, theta)
        elif sel == 'prior':
            p_id, res = self._add_prior_factor_pose(x, y, theta, prior_factor)
        elif sel == 'odometry':
            p_id, res = self._add_odometry_factor_pose(odometry_factor)

        self._pose_ids.append(p_id)

        return p_id, res

    def _add_no_factor_pose(self, x, y, theta):
        p_id = self._num_variables
        p_neo4j = json.dumps({'uid': p_id, 't': 'P'})
        self._num_variables += 1
        self._pose_to_odometry_or_prior[p_id] = []
        self._pose_to_measurements[p_id] = []
        res = self.session.run("MERGE (p:POSE:"+self._session_name+":NEWDATA { frtend: {pose_info} }) "
                               "RETURN id(p) as poseid",
                              {"pose_info": p_neo4j})
        return p_id, res

    def _add_prior_factor_pose(self, x, y, theta, prior_factor):
        if type(prior_factor) is not priorFactorPose2:
            raise TypeError("prior_factor must be of type priorFactorPose2")
        p_id = self._num_variables
        p_neo4j = json.dumps({'uid': p_id, 't': 'P'})
        self._num_variables += 1
        self._pose_to_odometry_or_prior[p_id] = []
        self._pose_to_measurements[p_id] = []
        p_prior_factor_neo4j = json.dumps({'meas': str(prior_factor.x) + ' ' + str(prior_factor.y) + ' ' + str(prior_factor.theta) + ' ' + str(prior_factor.noise.x) + ' 0 0 ' + str(prior_factor.noise.y) + ' 0 ' + str(prior_factor.noise.z),
                                            'lklh': 'PR2 G 3',
                                            'btwn': str(p_id),
                                            't': 'F'})
        self._pose_to_odometry_or_prior[p_id].append(self._max_factor_id)
        self._max_factor_id += 1
        res = self.session.run("MERGE (p:POSE:"+self._session_name+":NEWDATA { frtend: {pose_info} }) "
                               "MERGE (f:FACTOR:"+self._session_name+":NEWDATA { frtend: {prior_factor_info} }) "
                               "MERGE (p)-[:DEPENDENCE]-(f)"
                               "RETURN id(p) as poseid, id(f) as fctid",
                              {"pose_info": p_neo4j,
                               "prior_factor_info": p_prior_factor_neo4j})
        return p_id, res

    def _add_odometry_factor_pose(self, odometry_factor):
        if len(self._pose_ids) == 0:
            raise Exception("cannot add odometry-based pose because no prior poses exist")
        if type(odometry_factor) is not betweenFactorPose2:
            raise TypeError("odometry_factor must be of type betweenFactorPose2")
        p_id = self._num_variables
        head_id = self._pose_ids[-1]
        p_neo4j = json.dumps({'uid': p_id, 't': 'P'})
        head_neo4j = json.dumps({'uid': head_id, 't': 'P'})
        self._num_variables += 1
        self._pose_to_odometry_or_prior[p_id] = []
        self._pose_to_measurements[p_id] = []
        p_odometry_factor_neo4j = json.dumps({'meas': str(odometry_factor.delta_surge) + ' ' + str(odometry_factor.delta_sway) + ' ' + str(odometry_factor.delta_theta) + ' ' + str(odometry_factor.noise.x) + ' 0 0 ' + str(odometry_factor.noise.y) + ' 0 ' + str(odometry_factor.noise.z),
                                              'lklh': 'PP2 G 3',
                                              'btwn': str(head_id) + ' ' + str(p_id),
                                              't': 'F'})
        self._pose_to_odometry_or_prior[p_id].append(self._max_factor_id)
        self._max_factor_id += 1
        result = self.session.run("MERGE (p1:POSE:"+self._session_name+":NEWDATA { frtend: {previous_pose_info} }) "
                                  "MERGE (p2:POSE:"+self._session_name+":NEWDATA { frtend: {pose_info} })"
                                  "MERGE (f:FACTOR:"+self._session_name+":NEWDATA { frtend: {odometry_factor_info} }) "
                                  "MERGE (p1)-[:DEPENDENCE]-(f) " # add relationships
                                  "MERGE (p2)-[:DEPENDENCE]-(f) "
                                  "RETURN id(p1) as pose1id, id(p2) as pose2id, id(f) as fctid",
                                 {"previous_pose_info": head_neo4j,
                                  "pose_info": p_neo4j,
                                  "odometry_factor_info": p_odometry_factor_neo4j})
        return p_id, result

    # adds a measurement factor between a pose and landmark
    def add_measurement(self, measurement_factor, pose2_id=None):
        if pose2_id is None:
            p_id = self._pose_ids[-1]
        else:
            p_id = pose2_id
        p_neo4j = json.dumps({'uid': p_id, 't': 'P'})
        l_id = measurement_factor.landmark_id
        l_neo4j = json.dumps({'tag_id': l_id, 'uid': l_id, 't': 'L'})
        if type(measurement_factor) is rangeMeasurementFactorPose2Point2:
            p_measurement_factor_neo4j = json.dumps({'range': measurement_factor.range_val,
                                                     'lklh': 'rangeMEAS',
                                                     'btwn': str(p_id) + ' ' + str(measurement_factor.landmark_id),
                                                     't': 'F'})
            self._pose_to_measurements[p_id].append(self._max_factor_id)
            self._max_factor_id += 1
        elif type(measurement_factor) is bearingRangeMeasurementFactorPose2Point2:
            p_measurement_factor_neo4j = json.dumps({'range': measurement_factor.range_val,
                                                     'bearing': measurement_factor.bearing_val,
                                                     'lklh': 'rangeBearingMEAS',
                                                     'btwn': str(p_id) + ' ' + str(measurement_factor.landmark_id),
                                                     't': 'F'})
            self._pose_to_measurements[p_id].append(self._max_factor_id)
            self._max_factor_id += 1
        elif type(measurement_factor) is rangeFactorPose2Point2:
            p_measurement_factor_neo4j = json.dumps({'meas': str(measurement_factor.range_val) + str(measurement_factor.noise.x),
                                                     'lklh': 'R G 1',
                                                     'btwn': str(p_id) + ' ' + str(measurement_factor.landmark_id),
                                                     't': 'F'})
            self._pose_to_measurements[p_id].append(self._max_factor_id)
            self._max_factor_id += 1
        elif type(measurement_factor) is bearingRangeFactorPose2Point2:
            # TODO -- noise xx 0 yy, where is xy
            p_measurement_factor_neo4j = json.dumps({'meas': str(measurement_factor.bearing_val) + ' ' + str(measurement_factor.range_val) + ' ' + str(measurement_factor.noise.x) + ' 0 ' + str(measurement_factor.noise.y),
                                                     'lklh': 'BR G 2',
                                                     'btwn': str(p_id) + ' ' + str(measurement_factor.landmark_id),
                                                     't': 'F'})
            self._pose_to_measurements[p_id].append(self._max_factor_id)
            self._max_factor_id += 1
        else:
            raise TypeError("measurement_factor must be of type rangeFactorPose2Point2, bearingRangeFactorPose2Point2, rangeMeasurementFactorPose2Point2, or bearingRangeMeasurementFactorPose2Point2")

        self.session.run("MERGE (l:LANDMARK:"+self._session_name+":NEWDATA { frtend: {landmark_info} }) "
                         "MERGE (p:POSE:"+self._session_name+":NEWDATA { frtend: {pose_info} })"
                         "MERGE (f:FACTOR:"+self._session_name+":NEWDATA { frtend: {measurement_factor_info} }) "
                         "MERGE (l)-[:DEPENDENCE]-(f) " # add relationships
                         "MERGE (p)-[:DEPENDENCE]-(f) ",
                        {"landmark_info": l_neo4j,
                         "pose_info": p_neo4j,
                         "measurement_factor_info": p_measurement_factor_neo4j})


    def addmongokeys(self, neoid, oidname, newoid):
        res = self.session.run("MATCH (n:"+self._session_name+") " # finds if not exist
                               "WHERE id(n)="+str(neoid)+" "
                               "return n.frtend as frtend, n.mongo_keys as mongo_keys")

        elem = res.single()
        var_json_str1 = elem["frtend"]
        mymoids = elem["mongo_keys"]
        if mymoids == None:
            mymoids = {}
        else:
            mymoids = json.loads(mymoids)

        mymoids[oidname] = newoid
        mym_json_str = json.dumps(mymoids)

        # import IPython; IPython.embed()

        res = self.session.run("MATCH (n:"+self._session_name+") "
                               "WHERE id(n)="+str(neoid)+" "
                               "SET n.mongo_keys = {mym_info} "
                               "RETURN count(n) as numupdated",
                               {"mym_info": mym_json_str} )

        elem = res.single()
        if elem["numupdated"] != 1:
            ValueError("ERROR, did not work. Number of updated entries are = "+str(elem["numupdated"]) )
            return False
        return True



#
