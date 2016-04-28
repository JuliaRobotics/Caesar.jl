import socket
import sys
import numpy as np
from numpy import linalg as npla

from bot_geometry.rigid_transform import RigidTransform, Pose
from bot_geometry.quaternion import Quaternion
from bot_externals.draw_utils import publish_pose_list, publish_sensor_frame, publish_cloud, \
    publish_line_segments

def triggerPose(Dx, dt, distrule=1.5, rotrule=np.pi/4, timerule=30):
    dist = npla.norm(Dx.tvec)
    rotang = 2.0*np.arccos(Dx.quat[3])
    if dist >= distrule:
        return 1
    if rotang >= rotrule:
        return 2
    if dt >= timerule:
        return 3

def advOdoByRules(slam, prevpose, dx=None, newpose=None, previ=0, i=0, OFsum=-1):
    psname = ''
    if not dx and not newpose:
        raise 'advOdoByRule -- is not going to work without dx or newpose'
    if not dx:
        dx = prevpose.inverse().oplus(newpose)
    if triggerPose(dx, i-previ, timerule=600):
        if OFsum != -1:
            print 'Optical flow sum', OFsum
        meas = dx.to_roll_pitch_yaw_x_y_z()
        if OFsum == -1 or OFsum > 300000:
            mlst = [meas[3],meas[4],meas[2]]
        else:
            print 'Looks like the robot is not moving!'
            mlst = [0.0,0.0,meas[2]]
        psname = slam.addOdo(mlst)
        slam.redrawAll()
        return newpose, i, psname
    return prevpose, previ, psname


def readlines(sock, recv_buffer=4096, delim='\n'):
	buffer = ''
	data = True
	while data:
		data = sock.recv(recv_buffer)
		buffer += data
		while buffer.find(delim) != -1:
			line, buffer = buffer.split('\n', 1)
			yield line
	return

class NPSLAMWrapper(object):
    def __init__(self):
        # Create a TCP/IP socket
        self.cl = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.pose_nodes = dict()
        self.lastposeid = 1
        self.landm_nodes = dict()
        self.lastlndmkid = 1

    def connectServer(self, addr='localhost', port=60001):
        # Connect the socket to the port where the server is listening
        self.server_address = (addr, port)
        print >>sys.stderr, 'connecting to %s port %s' % self.server_address
        self.cl.connect(self.server_address)

    def disconnectServer(self):
        self.cl.close()

    def sendCmd(self, cmd):
        self.cl.sendall(cmd+'\n')
        if cmd == "QUIT":
            self.cl.close()
        else:
            for line in readlines(self.cl):
                return line

    def getParticles(self, lbl):
        res = self.sendCmd('GETPARTICLES '+lbl)
        rows = res[:-1].split(';')
        arrs = [] #(np.fromstring(rows[0], dtype=float, sep=','))
        for i in range(0,len(rows)):
            arrs.append(np.fromstring(rows[i], dtype=float, sep=',')) #V = np.hstack( (V, ))
        V = np.matrix(arrs)
        return V

    def drawParticles2d(self, V, frame_id='origin', name='particles', lname='yaw', c=['b'], lc=['r'], ll=0.05):
        LD = []
        if len(V) < 1:
            print 'drawParticles2d -- no data? skip'
            return
        d,N = V[0].shape
        l = len(V)
        for v in V:
            Dv = np.vstack((v[0:2,:], np.zeros((1,N)))).T
            LD.append(Dv)
        publish_cloud(name, LD, c=c, frame_id=frame_id)
        if d > 2:
            count = -1
            DV = np.zeros((d,l*N))
            DT = np.zeros((d,l*N))
            for v in V:
                tt = np.zeros((d,N))
                for i in range(0,N):
                    t = Pose.from_rigid_transform(0,RigidTransform(xyzw=Quaternion.from_roll_pitch_yaw(0,0,v[2,i]).to_xyzw(), tvec=[v[0,i], v[1,i], 0]))
                    pert = Pose.from_rigid_transform(0,RigidTransform(tvec=[ll,0,0]))
                    tv = (t.oplus(pert)).tvec
                    tt[0:2,i] = tv[0:2]
                count +=1
                DT[0:d,(count*N):((count+1)*N)] = tt
                Dv = np.vstack((v[0:2,:], np.zeros((1,N))))
                DV[0:d,(count*N):((count+1)*N)] = Dv
            publish_line_segments(lname, DV.T, DT.T, c=lc[0], frame_id=frame_id)


    def addOdo(self, z, frm=None, to=None, noise=np.array([0.08, 0, 0, 0.03, 0, 0.03]) ):
        frmid = frm
        if not frm:
            frmid = self.lastposeid
        if not frm and not to:
            frmid = self.lastposeid
            while self.lastposeid < self.lastlndmkid:
                self.lastposeid += 1
            self.lastposeid += 2
            to = self.lastposeid
        # next pose
        pn = self.sendCmd('ODOMETRY '+str(frmid)+' '+str(to)+' '+str(z[0])+' '+str(z[1])+' '+str(z[2])
                     +' '+str(noise[0])+' '+str(noise[1])+' '+str(noise[2])+' '+str(noise[3])
                     +' '+str(noise[4])+' '+str(noise[5]))
        return pn#self.lastposeid

    def addLandmBR(self, z, frmid=None, to=None, noise=[0.02, 0, 0.4]):
        if not frmid:
            frmid = self.lastposeid
        # if not to:
        #     while self.lastlndmkid < self.lastposeid:
        #         self.lastlndmkid += 1
        #     self.lastlndmkid += 2
        #     to = self.lastlndmkid
        if not to:
            to = self.getNextID()
        sendstr = 'LANDMBR '+str(frmid)+' '+str(to)+' '+str(z[0])+' '+str(z[1])+' '+str(noise[0])+' '+str(noise[1])+' '+str(noise[2])
        print sendstr
        retn = self.sendCmd(sendstr)
        return retn#self.lastlndmkid

    def addLandmBRMM(self, z, frmid, to1=None, to2=None, w=[0.5,0.5],noise=[0.02, 0, 0.4]):
        if not frmid:
            frmid = self.lastposeid
        if not to1 and not to2:
            print 'ERROR need to define to1 and to2'
            return
        sendstr = 'LANDMBRMM '+str(frmid)+' '+str(to1)+' '+str(w[0])+' '+str(to2)+' '+str(w[1])+' '+str(z[0])+' '+str(z[1])+' '+str(noise[0])+' '+str(noise[1])+' '+str(noise[2])
        print sendstr
        return self.sendCmd(sendstr)

    def addLandmBRAuto(self, z, frmid=None, to=None, noise=[0.02, 0, 0.4]):
        if not frmid:
            frmid = self.lastposeid
        if not to:
            to = '*'
        sendstr = 'LANDMBRAUTO '+str(frmid)+' '+str(to)+' '+str(z[0])+' '+str(z[1])+' '+str(noise[0])+' '+str(noise[1])+' '+str(noise[2])
        return self.sendCmd(sendstr)

    def init(self):
        self.sendCmd('INIT')

    def close(self):
        self.sendCmd('QUIT')

    def reset(self):
        self.sendCmd('RESET')

    def batchSolve(self):
        self.sendCmd('BATCHSOLVE')

    def ls(self, poses=True,landmarks=True):
        retstrs = (self.sendCmd('LS')).split(';')
        poses = retstrs[0].split(',')
        landm = retstrs[1].split(',')
        if landm[-1] == '':
            del landm[-1]
        return poses, landm

    def getID(self, lbl):
        # print 'getID', lbl
        idt = self.sendCmd('GETID '+lbl)
        # print 'and get back idt=', idt
        return int(idt)

    def getNextID(self):
        idt = self.sendCmd('GETNEXTID')
        return int(idt)

    def drawFactorGraphpdf(self):
        self.sendCmd('DRAWFGPDF')

    def redrawAll(self, poses=True,landmarks=True, posec='g', landmc='c', pname='poses', lname='landmarks'):
        poses, landm = self.ls()
        print 'drawing poses', poses
        Vx = []
        Cx = []
        for p in poses:
            Vx.append(self.getParticles(p))
            Cx.append(posec)
        self.drawParticles2d(Vx, c=Cx, name=pname, lname=('yaw'+pname))
        Vl = []
        Cl = []
        for l in landm:
            Vl.append(self.getParticles(l))
            Cl.append(landmc)
        self.drawParticles2d(Vl, c=Cl, name=lname)
