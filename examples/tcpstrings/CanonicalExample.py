import numpy as np
from JLSLAMInterf import NPSLAMWrapper

bonoi = np.array([0.2, 0, 0, 0.1, 0, 0.08])

npslam = NPSLAMWrapper()
npslam.connectServer()
npslam.init() # x1
npslam.addLandmBR([+np.pi/2,5.0], npslam.getID('x1'))
npslam.addLandmBR([+0.32175,15.81], npslam.getID('x1'))
npslam.addOdo([50, 0, -np.pi/2], noise=bonoi)
npslam.addLandmBR([+np.pi,5.0], npslam.getID('x2'))
npslam.addOdo([50, 0, -np.pi/2], noise=bonoi)
npslam.addOdo([47.5, 0, -np.pi/2], noise=bonoi) #x4


npslam.drawFactorGraphpdf()
print npslam.ls()
npslam.redrawAll()
npslam.batchSolve()
npslam.redrawAll()

npslam.addOdo([55, 0, 0],noise=bonoi) # x5
npslam.addLandmBRMM([-np.pi/2,5.0], npslam.getID('x5'), to1=npslam.getID('l1'), to2=npslam.getID('l2'))


npslam.drawFactorGraphpdf()
print npslam.ls()
npslam.redrawAll(pname='initp',posec='b',lname='landminit',landmc='r')
npslam.batchSolve()
npslam.redrawAll()

npslam.addOdo([45, 0, np.pi*3.0/4.0], noise=bonoi) #x6
npslam.addOdo([70.71, 0, np.pi*3.0/4.0], noise=bonoi) #x7
npslam.addOdo([57, 0, 0], noise=bonoi)#x8

npslam.drawFactorGraphpdf()
print npslam.ls()
npslam.batchSolve()
npslam.redrawAll(pname='poses2',lname='lndm2',posec='r')


# npslam.addLandmBRMM([+np.pi/2,5.0], npslam.getID('x8'), to=npslam.getID('l1'))
npslam.addLandmBRMM([+np.pi/2,5.0], npslam.getID('x8'), to1=npslam.getID('l1'), to2=npslam.getID('l2'))
# npslam.addLandmBR([+0.32175,15.81], npslam.getID('x8'), to=npslam.getID('l1'))
npslam.addOdo([40, 0, 0], noise=bonoi)#x9
npslam.addLandmBR([+np.pi/2,5.0], npslam.getID('x9'), to=npslam.getID('l3'))

npslam.drawFactorGraphpdf()
print npslam.ls()
npslam.batchSolve()
npslam.redrawAll(pname='poses3',lname='lndm3',posec='b')

# for i in range(1,11):
#     print ('x'+str(i)+'='), npslam.getParticles('x'+str(i)).mean(axis=1).T

npslam.sendCmd("QUIT")
npslam.disconnectServer()
