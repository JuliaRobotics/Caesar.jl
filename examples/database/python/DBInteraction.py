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

from neo4j.v1 import GraphDatabase, basic_auth # via queries
import json
import random#.uniform as ru

from pymongo import MongoClient
from bson import Binary
import uuid
##############

# To Run: 
# $ python NeoDBInteraction.py -f <path-to-rosbag>

# Make sure authfile path below exists where you run this script
# And the tags in the commands are the ones desired!

#authfile = '/home/rmata/neo_authfile.txt' # username on one line, password on next (for database)
#un,pw, addr = open(authfile).read().splitlines()

# Running Mongo

#client = MongoClient()

##############

class Neo4jTalkApp():
    def __init__(self):
        self.idx_ = 0 # odom_index

        ## Authentication and Setup for Neo4j
        authfile = '/home/rmata/neo_authfile.txt' # username on one line, password on next (for database)
        un,pw, addr = open(authfile).read().splitlines()
        self.driver = GraphDatabase.driver(addr, auth=basic_auth(un, pw))
        self.session = self.driver.session()
        self.session.run("MATCH (n:SESSTURTLE) DETACH DELETE n")
        self.old_odom = None
        self.odom_node_id = None # neo4j node id

        ## Authentication/Setup for Mongo
        client = MongoClient() # Default is local for now
        self.db = client.tester # test is the name of the data base?

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
        ########## gtsam.py
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
        poses = [pose_bc*pose_ct for pose_ct in poses_ct] # rigidtransforms for tag_ids

        for i in range(len(tag_ids)):
            rospy.logerr('\n\n tag is {}'.format(tag_ids[i]))
            self.session.run("MERGE (l1:POSE:NEWDATA:LAND:SESSTURTLE {frtend: {land_info}}) "
                             "MERGE (o1:POSE:NEWDATA:SESSTURTLE {frtend: {var_info}}) "
                             "MERGE (f:FACTOR:NEWDATA:SESSTURTLE {frtend: {fac_info}}) " # odom-landmark factor
                             "MERGE (o1)-[:REL]->(f) "
                             "MERGE (f)-[:REL]->(l1) "
                             "RETURN id(o1)",
                             {"land_info": json.dumps({"t":"P", "tag_id": tag_ids[i], "userready":0}), 
                              "var_info": json.dumps({"t":"P", "uid": self.idx_, "userready":0}), 
                              "fac_info": json.dumps({"t":"F", 
                                                      "meas":str(poses[i].t[0])+" "+str(poses[i].t[1])+\
                                                      " "+str(euler_from_quaternion(poses[i].xyzw)[2])+\
                                                      " 1e-3 0 1e-2" , "userready":0})})

    def on_odom_cb(self, data):
        #print "GETTING ODOM"
        r = RigidTransform(xyzw=[data.pose.pose.orientation.x, 
                                 data.pose.pose.orientation.y, 
                                 data.pose.pose.orientation.z, 
                                 data.pose.pose.orientation.w], 
                           tvec=[data.pose.pose.position.x, 
                                 data.pose.pose.position.y, 
                                 data.pose.pose.position.z])
        if self.old_odom:
            odom_diff = self.old_odom.inverse()*r
            running_result = self.session.run("MERGE (o1:POSE:NEWDATA:SESSTURTLE {frtend: {var_info1} }) " # finds/creates
                                              "MERGE (o2:POSE:NEWDATA:SESSTURTLE { frtend: {var_info2} })"
                                              "MERGE (f:FACTOR:NEWDATA:SESSTURTLE { frtend: {fac_info} }) " # odom-odom factor
                                              "MERGE (o1)-[:REL]-(f) " # add relationships
                                              "MERGE (o2)-[:REL]-(f) "
                                              "RETURN id(o1) as oid",
                                              {"var_info1":json.dumps({"t":"P", "uid":self.idx_, "userready":0}), 
                                               "var_info2":json.dumps({"t":"P", "uid":self.idx_+1, 
                                                                       "userready":0}), 
                                               "fac_info":json.dumps({"t":"F", "lklh":"PP2 G 3", 
                                                                      "meas":str(odom_diff.t[0])+" "+str(odom_diff.t[1])+" "+ \
                                                                      str(euler_from_quaternion(odom_diff.xyzw)[2]) + \
                                                                      " 1e-3 0 0 1e-3 0 5e-5",  
                                                                      "userready": 0,
                                                                      "btwn": str(self.idx_) + " " + str(self.idx_+1)})})
            for record in running_result: # just one record
                self.odom_node_id = record["oid"]
            self.idx_ += 1
        else:
            self.session.run("MERGE (o1:POSE:NEWDATA:SESSTURTLE {frtend: {var_info1} }) "
                             "MERGE (f:FACTOR:NEWDATA:SESSTURTLE { frtend: {fac_info} }) "
                             "MERGE (o1)-[:REL]-(f) ",
                             {"var_info1":json.dumps({"t":"P", "uid":self.idx_, "userready":0}),
                              "fac_info":json.dumps({"meas": "0 0 0 1e-4 0 0 1e-4 0 4e-6",
                                                     "t": "F", "lklh":"PR2 G 3",
                                                     "btwn": "0"})})
                              
        self.old_odom = r

    def on_keyframe_cb(self, data):
        im = data.data # should be under 16 MB
        ## TODO Get BigData key?
        new_im_uid = uuid.uuid4()
        #rospy.logwarn(str(new_im_uid.hex))
        self.session.run("MATCH (od:POSE:NEWDATA:SESSTURTLE)" 
                         "WHERE id(od)={jungle}" 
                         "SET od += {newkeys}",
                         {"jungle":self.odom_node_id,
                          "newkeys":{"mongo_keys":json.dumps({"keyframe_rgb":str(new_im_uid.hex)})}})
        j = Binary(im)
        self.db.collection.insert({"key": str(new_im_uid.hex),
                                   "rgb_image":j})

if __name__=="__main__":
    m = Neo4jTalkApp()
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
    
    args = parser.parse_args()

    # Setup dataset/log
    dataset = ROSBagReader(filename=os.path.expanduser(args.filename), 
                           decoder=[
                               Decoder(channel=args.odom_channel, every_k_frames=10), # internal throttling...
                               Decoder(channel=args.tag_channel, every_k_frames=1),
                               Decoder(channel=args.keyframe_channel, every_k_frames=1)
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
