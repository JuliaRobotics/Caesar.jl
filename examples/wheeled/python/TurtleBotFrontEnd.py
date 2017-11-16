import numpy as np
import cv2
from JLSLAMInterf import NPSLAMWrapper, advOdoByRules
from JLBayesTrackerInterf import BayesFeatureTracking
import argparse
from bot_geometry.rigid_transform import RigidTransform, Pose
from bot_externals.ros.bag_utils import ROSBagReader, ROSBagController, \
    Decoder, ImageDecoder, NavMsgDecoder, GazeboDecoder

import bot_externals.draw_utils as draw_utils
from pybot_apriltags import AprilTag, AprilTagsWrapper

from multiprocessing import Process, Queue

def areaTriangle(a,b,C):
    return 0.5*a*b*np.sin(C)

def draw_flow(img, flow, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1)
    fx, fy = flow[y,x].T
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.polylines(vis, lines, 0, (0, 255, 0))
    for (x1, y1), (x2, y2) in lines:
        cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis

def doDrawFlow(q, prevgray, gray, step=16):
    flow = cv2.calcOpticalFlowFarneback(prevgray, gray, 0.5, 3, 15, 3, 5, 1.2, 0)
    viz = draw_flow(gray, flow)
    flowsum = np.sum(np.absolute(flow))
    q.put([viz,flow, flowsum])

class TurtlebotROSSLAM(ROSBagController):
    def __init__(self, dataset):
        """

        Queues:
            odom_q: Odometry queue (wrt body)
            im_q: RGB Image queue
            depth_im_q: Depth Image queue

        Poses:
            p_cb: camera and baseline
            p_rd: rgb and depth frame

        """

        ROSBagController.__init__(self, dataset)
        # NPSLAMWrapper.__init__(self)
        self.npslam = NPSLAMWrapper()
        self.odocounter = 0
        self.prevodocount = 0
        self.sinceSLAMSolve = 1
        self.prevpose = Pose(RigidTransform())
        self.at = AprilTagsWrapper(tag_size=0.2, fx=554.2547, fy=554.2547, cx=320.5, cy=240.5)
        self.gtpsCnt = 0
        self.odomsgCnt = 0
        self.gt = RigidTransform.from_homogenous_matrix(np.eye(4))
        self.bTc = RigidTransform.from_angle_axis(np.pi/2,(0,0,-1),(0,0,0)).oplus(RigidTransform.from_angle_axis(np.pi/2,(-1,0,0),(0,0,0)))
        self.tbpose = RigidTransform.from_homogenous_matrix(np.eye(4))
        self.prevtrckpose = RigidTransform.from_homogenous_matrix(np.eye(4))
        self.reduceFeatTracking = 0
        self.trackedFeats = {}
        self.localLmIDs = {}
        self.gtcam = self.bTc
        self.odocam = self.bTc
        self.lastposename = ''
        self.tagwposes = []
        self.avgdBR = []
        self.firstpass = True
        self.flow = None
        self.flowsum = 0.0
        self.accrueCount = 0.0
        self.accrueFlow = 0.0
        # BayesFeatureTracking
        self.bft = BayesFeatureTracking()
        self.currFeatTracks = {}
        self.lastimg = np.zeros((480,640,3), np.uint8)
        # Init node and tf listener
        # -----------------
        print('Initializing class {:s}'.format(self.__class__.__name__))
        # self.subscribe('/gazebo/model_states', self.on_gt)


    def procFeatSightsDraw(self):
        # print 'procFeatTracks -- current tags'
        frm = []
        totag = []
        self.BR = []
        draw = False
        for t in self.tags:
            draw = True
            rtt = RigidTransform.from_homogenous_matrix(t.getPose())
            t = rtt.to_roll_pitch_yaw_x_y_z()
            # print 'tag t', t
            rang = np.sqrt(t[5]**2 + t[3]**2)
            bear = np.arctan2(-t[3], t[5])
            self.BR.append(np.array([bear, rang]))
            po = self.tbpose.to_roll_pitch_yaw_x_y_z()
            pp = np.array([self.odocam.translation[0],self.odocam.translation[1],0])
            tpp = np.array([pp[0]+rang*np.cos(bear+po[2]), pp[1]+rang*np.sin(bear+po[2]), 0])
            frm.append( pp )
            totag.append( tpp )
        if draw:
            FRM = np.vstack(frm)
            TOTAG = np.vstack(totag)
            draw_utils.publish_line_segments('tagBR', FRM, TOTAG, c='b', frame_id='origin', reset=True)

    def avgNearTagSights(self, BR, tol=0.32):
        l = len(BR)
        self.avgdBR = []
        skiplst = []
        for i in range(l):
            if i not in skiplst:
                # print 'bear', BR[i]
                avgb, avgr = 0, 0
                N = 0
                for j in range(i,l):
                    if areaTriangle(BR[i][1],BR[j][1],abs(BR[i][0] - BR[j][0])) < tol:
                        # print '   yes close', BR[j]
                        skiplst.append(j)
                        avgb += BR[j][0]
                        avgr += BR[j][1]
                        N += 1
                B, R = avgb/N, avgr/N
                self.avgdBR.append(np.array([B, R]))

    def do_feat_tracking(self, BR, dOdo):
        t = dOdo.to_roll_pitch_yaw_x_y_z()
        dx = [t[3],t[4],t[2]]
        r = []
        draw = False
        draw2 = False
        frm = []
        totag = []
        frm2 = []
        totrck = []
        po = self.tbpose.to_roll_pitch_yaw_x_y_z()
        pp = np.array([self.odocam.translation[0],self.odocam.translation[1],0])
        for i in range(len(BR)):
            bear = BR[i][0]
            rang = BR[i][1]
            r.append([rang,bear,1])
            # po = self.tbpose.to_roll_pitch_yaw_x_y_z()
            # pp = np.array([self.odocam.translation[0],self.odocam.translation[1],0])
            tpp = np.array([pp[0]+rang*np.cos(bear+po[2]), pp[1]+rang*np.sin(bear+po[2]), 0])
            frm.append( pp )
            totag.append( tpp )
            draw=True
        if draw:
            FRM = np.vstack(frm)
            TOTAG = np.vstack(totag)
            draw_utils.publish_line_segments('avgdBR', FRM, TOTAG, c='r', frame_id='origin', reset=True)
        retd = self.bft.processSightings(dx, r)
        self.trackedFeats = retd
        if retd:
            for i in retd:
                bear = retd[i][0]
                rang = retd[i][1]
                tpp = np.array([pp[0]+rang*np.cos(bear+po[2]), pp[1]+rang*np.sin(bear+po[2]), 0])
                frm2.append( pp )
                totrck.append( tpp )
                draw2 = True
            if draw2:
                FRM = np.vstack(frm2)
                TOTAG = np.vstack(totrck)
                draw_utils.publish_line_segments('trckBR', FRM, TOTAG, c='g', frame_id='origin', reset=True)
        # print 'feattrack retd', retd

    def on_depth(self, t, data):
        """
        Callback for images:
        Only responsible for accumulating images, and maintaining
        queue for images
        """
        print 'ot depth', data.shape, data.dtype

    def on_image(self, t, data):
        """
        Callback for images:
        Only responsible for accumulating images, and maintaining
        queue for images
        """
        q = Queue()
        p1 = None
        # cv2.imshow('data',data)
        # cv2.waitKey()
        self.lastimg = data
        gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
        if not self.firstpass:
            # flow = cv2.calcOpticalFlowFarneback(self.prevgray, gray, 0.5, 3, 15, 3, 5, 1.2, 0)
            # self.prevgray = gray
            #cv2.imshow('flow', draw_flow(gray, flow))
            p1 = Process(target=doDrawFlow, args=(q, self.prevgray, gray))
            p1.start()
        # Do tag detections
        self.tags = self.at.process(gray, return_poses=True)
        self.tagwposes= []
        for t in self.tags:
            tagpose = RigidTransform.from_homogenous_matrix(t.getPose())
            wpose = self.odocam.oplus(tagpose) # tbpose
            self.tagwposes.append(  wpose  )
        ids = [str(t.id) for t in self.tags]
        # print len(self.tagwposes)
        self.procFeatSightsDraw()
        self.avgNearTagSights(self.BR)
        draw_utils.publish_pose_list('apriltag', self.tagwposes, frame_id='origin', texts=ids)
        cv2.imshow('current',data)
        if not self.firstpass:
            vizflow = q.get(True)
            p1.join()
            viz = vizflow[0]
            self.flow = vizflow[1]
            self.flowsum = vizflow[2]
            cv2.imshow('flow', viz)
            # gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
            # flow = cv2.calcOpticalFlowFarneback(self.prevgray, gray, 0.5, 3, 15, 3, 5, 1.2, 0)
            self.prevgray = gray
            # cv2.imshow('flow', draw_flow(gray, flow))
        else:
            self.prevgray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
            self.firstpass = False
        cv2.waitKey(1)


    def procFeatToLandms(self):
        psid = self.npslam.getID(self.lastposename)
        for i in range(len(self.avgdBR)):
            B,R = self.avgdBR[i][0], self.avgdBR[i][1]
            print 'Add landmark', B, ', ', R, 'from', self.lastposename, 'psid=',psid
            self.npslam.addLandmBRAuto([B, R], psid) #addLandmBRAuto
        print ''

    def procTrackedFeatToLandms(self):
        psid = self.npslam.getID(self.lastposename)
        for i in self.trackedFeats:
            B,R = self.trackedFeats[i][0], self.trackedFeats[i][1]
            print 'Add landmark B=', B, ', R=', R, 'from pose=', self.lastposename, 'psid=',psid, 'key i=',i
            if not self.localLmIDs.has_key(i):
                retlb = self.npslam.addLandmBRAuto([B, R], psid) #addLandmBRAuto, addLandmBR
                self.localLmIDs[i] = self.npslam.getID(retlb)
                print 'Adding new landmark i=', i, 'retlb=', retlb, 'gets SLAM ID=', self.localLmIDs[i]
            else:
                print 'Found previous landmark', i, 'trying to reuse to=',self.localLmIDs[i]
                self.npslam.addLandmBRAuto([B, R], psid, to=self.localLmIDs[i]) #addLandmBR
        print ''


    def on_odom(self, t, tbpose):
        self.tbpose = tbpose
        self.odocam = tbpose.oplus(self.bTc)
        self.odomsgCnt += 1
        draw_utils.publish_pose_list('odocam', [Pose.from_rigid_transform(self.odomsgCnt, self.odocam)], frame_id='origin')
        if self.odocounter > 1:
            accrueFlow = self.accrueFlow
        else:
            accrueFlow = 9999999
        self.odocounter += 1
        self.prevpose, self.prevodocount, self.lastposename = advOdoByRules(self.npslam, self.prevpose, newpose=tbpose, previ=self.prevodocount, i=self.odocounter, OFsum=accrueFlow)
        self.reduceFeatTracking += 1
        if self.reduceFeatTracking % 5 == 0:
            dx = self.prevtrckpose.inverse().oplus(tbpose)
            self.prevtrckpose = tbpose
            self.do_feat_tracking(self.avgdBR, dx)
            self.reduceFeatTracking = 0
            self.accrueCount += 1.0
            self.accrueFlow = (self.accrueFlow*(self.accrueCount-1.0) + self.flowsum)/self.accrueCount
        if self.prevodocount == self.odocounter:
            self.sinceSLAMSolve += 1
            self.procTrackedFeatToLandms() #procFeatToLandms()
            # self.npslam.drawFactorGraphpdf()
            self.accrueCount = 0.0
            self.accrueFlow = 0.0
            # new pose, save keyframe image
            cv2.imshow('lastsaved',self.lastimg)
            filename ='/home/dehann/software/pyslam-pod/examples/images/keyframe_'+str(self.lastposename)+'.png'
            print 'SAVING TO', filename
            cv2.imwrite(filename,self.lastimg)
        if self.sinceSLAMSolve % 5 == 0:
            # self.npslam.batchSolve() # when using local dictionaries
            self.npslam.setReadyForSolve() # when using Database
            self.sinceSLAMSolve = 1
            # self.npslam.redrawAll()


    def on_gt(self, t, data):
        # print('on_gt', data)
        # gtpose = RigidTransform.from_homogenous_matrix(t.getPose())
        # print type(t), t, type(data)
        self.gtpsCnt += 1
        self.gt = data
        self.gtcam = data.oplus(self.bTc)
        draw_utils.publish_pose_list('gtcam', [Pose.from_rigid_transform(self.gtpsCnt, self.gtcam)], frame_id='origin')

        # draw_utils.publish_pose_list('GT_pose', [Pose.from_rigid_transform(self.gt_counter.index, data)],
        #                              frame_id='origin', reset=self.gt_counter.index == 0)






# npslam = NPSLAMWrapper()
# npslam.connectServer()#addr='mrg-liljon.csail.mit.edu')
# npslam.init() # x1
# # npslam.addLandmBR([+np.pi/2,5.0], npslam.getID('x1'))
# # npslam.addLandmBR([+0.32175,15.81], npslam.getID('x1'))
#
# print npslam.ls()
# npslam.batchSolve()
#
# npslam.sendCmd("QUIT")
# npslam.disconnectServer()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='rosbag player')
    parser.add_argument(
        '-f', '--filename', type=str, required=True,
        help="Filename: rosbag (.bag)")
    parser.add_argument(
        '-o', '--odom', type=str, default='/odom', required=False,
        help="Odometry (channel)")
    parser.add_argument(
        '-i', '--image', type=str, default='/camera/rgb/image_raw', required=False,
        help="Image (channel)")
    parser.add_argument(
        '-S', '--SLAM', type=str, default='localhost', required=False,
        help="NPSLAM server address")
    parser.add_argument(
        '-T', '--track', type=str, default='localhost', required=False,
        help="BayesFeatureTracking server address")
    args = parser.parse_args()
    print np.version.version
    # Setup dataset/log
    dataset = ROSBagReader(filename=args.filename,
                           decoder=[
                               GazeboDecoder(),
                               ImageDecoder(channel=args.image, scale=1, every_k_frames=1),
                               NavMsgDecoder(channel=args.odom, every_k_frames=1),
                               ImageDecoder(channel='/camera/depth/image_raw', scale=1, encoding='passthrough', every_k_frames=1)
                           ],
                           every_k_frames=1, start_idx=0, index=False, verbose=False)

    # Setup TurtlebotROS
    app = TurtlebotROSSLAM(dataset)
    # register callbacks
    app.subscribe(args.image, app.on_image)
    app.subscribe(args.odom, app.on_odom)
    app.subscribe('/gazebo/model_states', app.on_gt)
    app.subscribe('/camera/depth/image_raw', app.on_depth)
    # connect to SLAM server
    app.npslam.connectServer(addr=args.SLAM)
    app.npslam.init() # x1
    # connect to BayesFeatureTracking server
    app.bft.connectServer(addr=args.track)
    app.run()
    # app.npslam.batchSolve()
    app.npslam.setReadyForSolve
    app.sinceSLAMSolve = 1
    # app.npslam.redrawAll()
    app.npslam.close()
    app.bft.close()
    cv2.destroyAllWindows()
