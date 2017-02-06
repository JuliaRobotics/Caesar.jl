from neo4j.v1 import GraphDatabase, basic_auth # via queries
import json
import random#.uniform as ru
import rospy
import math
import numpy as np
import argparse
import os
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#from pybot.geometry.rigid_transform import RigidTransform, Pose, Quaternion
from pybot.externals.ros.bag_utils import ROSBagReader, ROSBagController
from pybot.externals.ros.bag_utils import Decoder, ImageDecoder, NavMsgDecoder, TfDecoderAndPublisher
from pybot.utils.db_utils import AttrDict
from apriltags_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry

from neo4j.v1 import GraphDatabase, basic_auth # via queries
import json
import random#.uniform as ru

##############

# To Run: 
# $ python NeoDBInteraction.py -f <path-to-rosbag>

# Make sure authfile path below exists where you run this script

authfile = '/home/rmata/neo_authfile.txt' # username on one line, password on next (for database)
un,pw = open(authfile).read().splitlines()

##############

class Neo4jTalkApp():
    def __init__(self):
        self.idx_ = 0 # odom_index
        authfile = '/home/rmata/neo_authfile.txt' # username on one line, password on next (for database)
        un,pw = open(authfile).read().splitlines()
        self.driver = GraphDatabase.driver("bolt://mrg-liljon.csail.mit.edu", auth=basic_auth(un, pw))
        self.session = self.driver.session()
        self.session.run("MATCH (n:SESSTURTLE) DETACH DELETE n")
        
    def on_tags_detection_cb(self, tag_array, tf_dict=None):
        #print "DETECTING TAGS"
        detections = tag_array.detections
        if len(detections) == 0:
            return # don't do anything
        tag_ids = [d.id for d in detections]
        frame_ct = tag_array.detections[0].pose.header.frame_id # frame of the tag in the camera ref
        rospy.logerr('Tag detections of {}'.format(tag_ids))
        poses_ct = [[[d.pose.pose.orientation.x,
                    d.pose.pose.orientation.y,
                     d.pose.pose.orientation.z,
                     d.pose.pose.orientation.w],
                    [d.pose.pose.position.x,
                     d.pose.pose.position.y,
                     d.pose.pose.position.z]] for d in detections]
        print poses_ct
        ########## gtsam.py
        # if not tf_dict:
        #     if (self.transformer.frameExists('/base_link') and self.transformer.frameExists(frame_ct)):
        #         t = self.transformer.getLatestCommonTime(frame_ct,'/base_link')
        #         try:
        #             (trans, rot) = self.transformer.lookupTransform('/base_link', frame_ct,  t)
        #             pose_bc = RigidTransform(xyzw=rot, tvec=trans) # pose of the camera in the body ref frame
        #         except:
        #             print sys.exc_info()
        #             rospy.logerr('could not get camera_rgb to base_link transform')
        #             return                
        # else:
        #     e = tf_dict[0]#tf_dict[from, to] = (trans, rot) in the form of a RigidTransform
        #     pose_bc = RigidTransform(xyzw=e.quat, tvec=e.tvec).inverse() #pose of camera in body frame
        # #for pose_ct in poses_ct:
        # poses = [(pose_bc*pose_ct).matrix for pose_ct in poses_ct] # rigidtransforms for tag_ids
        for i in range(len(tag_ids)):
            rospy.logerr('\n\n tag is {}'.format(tag_ids[i]))
            self.session.run("MERGE (l1:POSE:NEWDATA:LAND:SESSTURTLE {slam_info: {land_info}})"
                        "MERGE (o1:POSE:NEWDATA:ODOM:SESSTURTLE {slam_info: {var_info}})"
                        "MERGE (f:FACTOR:NEWDATA:SESSTURTLE {slam_info: {fac_info}})" # odom-landmark factor
                        "MERGE (o1)-[:REL]->(f) "
                        "MERGE (f)-[:REL]->(l1) ",
                             {"land_info": json.dumps({"tag_id": tag_ids[i], "userready":0}),#, "tag_coordinates": poses[i].tvec}), 
                              "var_info": json.dumps({"slam_id": self.idx_, "userready":0}), 
                              "fac_info": json.dumps({"camera_relative_coordinates": poses_ct[i][0], "userready":0})})

    def on_odom_cb(self, data):
        #print "GETTING ODOM"
        r = [[data.pose.pose.orientation.x, 
              data.pose.pose.orientation.y, 
              data.pose.pose.orientation.z, 
              data.pose.pose.orientation.w], 
             [data.pose.pose.position.x, 
              data.pose.pose.position.y, 
              data.pose.pose.position.z]]
        #if self.old_odom:
        # odom_diff = self.old_odom.inverse()*r
        self.session.run("MERGE (o1:POSE:ODOM:NEWDATA:SESSTURTLE {slam_info: {var_info1} }) " # finds/creates
                    "MERGE (o2:POSE:ODOM:NEWDATA:SESSTURTLE { slam_info: {var_info2} })"
                    "MERGE (f:FACTOR:NEWDATA:SESSTURTLE { slam_info: {fac_info} }) " # odom-odom factor
                    "MERGE (o1)-[:REL]->(f) " # add relationships
                    "MERGE (o2)-[:REL]->(f) ",
                         {"var_info1":json.dumps({"slam_id":self.idx_, "userready":0}), 
                          "var_info2":json.dumps({"slam_id":self.idx_+1, "userready":0}), 
                          "fac_info":json.dumps({"from": self.idx_, "to": self.idx_+1,"relative_coordinates": r[0], "userready":0})})#odom_diff})})
        self.idx_ += 1
        #self.old_odom = self.

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
    args = parser.parse_args()

    # Setup dataset/log
    dataset = ROSBagReader(filename=os.path.expanduser(args.filename), 
                           decoder=[
                               Decoder(channel=args.odom_channel, every_k_frames=10), # internal throttling...
                               Decoder(channel=args.tag_channel, every_k_frames=1),
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
    controller.subscribe(args.odom_channel, convert_odom(m.on_odom_cb))
    controller.subscribe(args.tag_channel, convert_tags(m.on_tags_detection_cb, d))
    controller.run()
