import json
import random#.uniform as ru
import rospy
import math
import numpy as np
import argparse
import os
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pybot.geometry.rigid_transform import RigidTransform, Pose, Quaternion
from pybot.externals.ros.bag_utils import ROSBagReader, ROSBagController
from pybot.externals.ros.bag_utils import Decoder, ImageDecoder, NavMsgDecoder, TfDecoderAndPublisher
from pybot.utils.db_utils import AttrDict
from apriltags_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
import cv2

from neo4j.v1 import GraphDatabase, basic_auth # via queries
import json
import random#.uniform as ru

from pymongo import MongoClient
from bson import Binary
# import uuid
##############

from neo4j_interact import neo4j_interact
from neo4j_interact import Noise1D, Noise2D, Noise3D
from neo4j_interact import priorFactorPoint2, priorFactorPose2, betweenFactorPose2, rangeFactorPose2Point2, bearingRangeFactorPose2Point2

# To Run:
# $ python NeoDBInteraction.py -f <path-to-rosbag> ("+self.sessname+" in particular is autonomous_2017-01-15-15-34-19.bag)

# Make sure authfile path below exists where you run this script
# And the labels/properties in the commands are the ones desired!

authfile = '/home/dehann/neo_authfile.txt' # username on one line, password on next (for database)

priorNoise = Noise3D(0.2, 0.2, 0.05)
landmarkPriorNoise = Noise2D(0.002, 0.002)
odometryNoise = Noise3D(0.1, 0.05, 0.01)
rangeNoise = Noise1D(0.25)
bearingRangeNoise = Noise2D(0.1, 0.25)

# Running Mongo
#client = MongoClient() # auth for

##############

class Neo4jTalkApp():
    def __init__(self, sessname):
        self.idx_ = 0 # odom_index
        self.sessname = sessname
        ## Authentication and Setup for Neo4j
        authfile = '/home/dehann/neo_authfile.txt' # username on one line, password on next (for database)
        self.neo4j_iface = neo4j_interact(authfile=authfile, session_name=sessname)
        self.odom_diff = None
        self.old_odom = None
        self.odom_node_id = None # neo4j node id

        ## Authentication/Setup for Mongo
        mongo_authfile = "/home/dehann/mongo_authfile.txt"
        maddr = open(mongo_authfile).read().splitlines()
        print maddr
        client = MongoClient(maddr) # Default is local for now
        self.db = client.CloudGraphs # test is the name of the data base

    def on_tags_detection_cb(self, tag_array, tf_dict=None):
        #print "DETECTING TAGS"
        detections = tag_array.detections
        if len(detections) == 0:
            return # don't do anything
        tag_ids = [d.id for d in detections]
        frame_ct = tag_array.detections[0].pose.header.frame_id # frame of the tag in the camera ref
        rospy.logerr('Tag detections of {}'.format(tag_ids))
        poses_ct =[RigidTransform(xyzw=[d.pose.pose.orientation.x,
                                         d.pose.pose.orientation.y,
                                         d.pose.pose.orientation.z,
                                         d.pose.pose.orientation.w],
                                   tvec=[d.pose.pose.position.x,
                                         d.pose.pose.position.y,
                                         d.pose.pose.position.z]) for d in detections]
        if not tf_dict:
            if (self.transformer.frameExists('/base_link') and self.transformer.frameExists(frame_ct)):
                t = self.transformer.getLatestCommonTime(frame_ct,'/base_link')
                try:
                    (trans, rot) = self.transformer.lookupTransform('/base_link', frame_ct,  t)
                    pose_bc = RigidTransform(xyzw=rot, tvec=trans) # pose of the camera in the body ref frame
                except:
                    print sys.exc_info()
                    rospy.logerr('could not get camera_rgb to base_link transform')
                    return
        else:
            e = tf_dict[0]#tf_dict[from, to] = (trans, rot) in the form of a RigidTransform
            pose_bc = RigidTransform(xyzw=e.quat, tvec=e.tvec).inverse() #pose of camera in body frame
        poses = [pose_bc*pose_ct for pose_ct in poses_ct] # rigidtransforms for tag_ids, in body frame

        for i in range(len(tag_ids)):
            x = poses[i].t[0]
            y = poses[i].t[1]
            #self.neo4j_iface.add_measurement(bearingRangeFactorPose2Point2(tag_ids[i], math.atan2(y,x), math.sqrt(x*x + y*y), bearingRangeNoise))

    def on_odom_cb(self, data):
        r = RigidTransform(xyzw=[data.pose.pose.orientation.x,
                                 data.pose.pose.orientation.y,
                                 data.pose.pose.orientation.z,
                                 data.pose.pose.orientation.w],
                           tvec=[data.pose.pose.position.x,
                                 data.pose.pose.position.y,
                                 data.pose.pose.position.z]) # current odom
        if not self.old_odom:
            self.old_odom = r
            # no odom diff set
        else:
            self.odom_diff = self.old_odom.inverse()*r
        #rospy.logwarn(str(self.odom_diff.t[0])+"  "+ str(self.odom_diff.t[1])+"  "+str(euler_from_quaternion(self.odom_diff.xyzw)[2]))


    def on_keyframe_cb(self, data):
        im = data # should be under 16 MB
        res, imdata = cv2.imencode('.png', im)
        oid = self.db["bindata"].insert({"neoNodeId": -1, "val": Binary(imdata.tostring()), "description": "Auto-inserted with DBInteraction.py"})

        # add odom
        if self.idx_ == 0:
            neo4j_iface.add_pose(0, 0, 0, priorFactorPose2(0, 0, 0, priorNoise), None)
        if self.odom_diff:
            odometry = betweenFactorPose2(self.odom_diff.t[0], self.odom_diff.t[1], euler_from_quaternion(self.odom_diff.xyzw)[2], odometryNoise)
            p_id, running_result = neo4j_iface.add_pose(None, None, None, None, odometry)
            # running_result = self.session.run("MERGE (o1:POSE:NEWDATA:"+self.sessname+" {frtend: {var_info1} }) " # finds/creates
            #                                   "MERGE (o2:POSE:NEWDATA:"+self.sessname+" { frtend: {var_info2} })"
            #                                   "MERGE (f:FACTOR:NEWDATA:"+self.sessname+" { frtend: {fac_info} }) " # odom-odom factor
            #                                   "MERGE (o1)-[:DEPENDENCE]-(f) " # add relationships
            #                                   "MERGE (o2)-[:DEPENDENCE]-(f) "
            #                                   "RETURN id(o1) as oid",
            #                                   {"var_info1":json.dumps({"t":"P", "uid":self.idx_, "userready":0}),
            #                                    "var_info2":json.dumps({"t":"P", "uid":self.idx_+1,
            #                                                            "userready":0}),
            #                                    "fac_info":json.dumps({"t":"F", "lklh":"PP2 G 3",
            #                                                           "meas":str(self.odom_diff.t[0])+" "+str(self.odom_diff.t[1])+" "+ \
            #                                                           str(euler_from_quaternion(self.odom_diff.xyzw)[2]) + \
            #                                                           " 1e-3 0 0 1e-3 0 5e-5",
            #                                                           "userready": 0,
            #                                                           "btwn": str(self.idx_) + " " + str(self.idx_+1)})})
            for record in running_result: # just one record
                self.odom_node_id = record["pose2id"]
            self.addmongokeys(self.odom_node_id, "keyframe_rgb", str(oid) )
            # self.session.run("MATCH (od:POSE:NEWDATA:"+self.sessname+")"
            #                  "WHERE id(od)={odom_node_id}"
            #                  "SET od += {newkeys}",
            #                  {"odom_node_id":self.odom_node_id,
            #                   "newkeys":{"mongo_keys":json.dumps({"keyframe_rgb":str(oid)})}})
            self.idx_ += 1
            self.old_odom = self.old_odom*self.odom_diff # advance old odom
            self.odom_diff = None # reset difference

if __name__=="__main__":
    # rospy.spin()
    parser = argparse.ArgumentParser(
        description='rosbag player')
    parser.add_argument(
        '-f', '--filename', type=str, required=True,
        help="Filename: rosbag (.bag)")
    parser.add_argument(
        '-t', '--tag-channel', type=str, required=False,
        default='/camera/rgb/tag_detections',
        help='/camera/rgb/tag_detections')
    parser.add_argument(
        '-o', '--odom-channel', type=str, required=False,
        default='/odom_throttle',
        help='/odom_throttle')
    parser.add_argument(
        '-k', '--keyframe-channel', type=str, required=False,
        default='/camera/rgb/image_raw/compressed_triggered',
        help='/camera/rgb/image_raw/compressed_triggered')
    parser.add_argument(
        '-S', '--sessionname', type=str, required=True,
        help='SESS?')

    args = parser.parse_args()

    m = Neo4jTalkApp(args.sessionname)

    # Setup dataset/log
    dataset = ROSBagReader(filename=os.path.expanduser(args.filename),
                           decoder=[
                               Decoder(channel=args.odom_channel, every_k_frames=1), # internal throttling
                               Decoder(channel=args.tag_channel, every_k_frames=1),
                               ImageDecoder(channel=args.keyframe_channel, every_k_frames=5,compressed=True)
                           ],
                           every_k_frames=1, start_idx=0, index=False)
    d = dataset.establish_tfs([('/camera_rgb_optical_frame', '/base_link')])

    # Create ROSBagController
    controller = ROSBagController(dataset)
    def convert_odom(func):
        def _func(t, d):
            return func(d)
        return _func

    def convert_tags(func, tf_dict):
        def _func(t,d):
            return func(d, tf_dict)
        return _func

    def convert_keyfr(func):
        def _func(t, d):
            return func(d)
        return _func

    # Subscribe to callbacks that modify the databases
    controller.subscribe(args.odom_channel, convert_odom(m.on_odom_cb))
    controller.subscribe(args.tag_channel, convert_tags(m.on_tags_detection_cb, d))
    controller.subscribe(args.keyframe_channel, convert_keyfr(m.on_keyframe_cb))
    controller.run()
