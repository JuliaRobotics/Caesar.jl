import numpy as np
from JLSLAMInterf import NPSLAMWrapper

npslam = NPSLAMWrapper()
npslam.connectServer()
npslam.init()
v2 = npslam.addOdo([5, 0, 0])
v3 = npslam.addOdo([5, 0, 0])
v4 = npslam.addOdo([5, 0, 0])

l1 = npslam.addLandmBR([-np.pi/4,np.sqrt(50.0)], v2 ,noise=[0.05, 0, 0.5])
npslam.addLandmBR([-np.pi/2,5.0], v3, to=l1)
npslam.addLandmBR([-3*np.pi/4,np.sqrt(50.0)], v4, to=l1, noise=[0.05, 0, 0.1])

npslam.drawFactorGraphpdf()

print npslam.ls()
npslam.redrawAll(pname='initp',posec='b',lname='landminit',landmc='r')
npslam.solveTree()
npslam.redrawAll()

v5 = npslam.addOdo([5, 0, -np.pi/2])
v6 = npslam.addOdo([30, 0, -np.pi/2])
v7 = npslam.addOdo([20.0, 0, 0.0])
v8 = npslam.addOdo([20.0, 0, -np.pi/2])
v9 = npslam.addOdo([30.0, 0, -np.pi/2])
v10 = npslam.addOdo([30.0, 0, 0])
npslam.addLandmBR([-np.pi/2,5.0], npslam.getID('x10'), to=l1)
npslam.drawFactorGraphpdf()

for i in range(1,11):
    print ('x'+str(i)+'='), npslam.getParticles('x'+str(i)).mean(axis=1).T

print npslam.ls()
npslam.redrawAll(pname='initp',posec='b',lname='landminit',landmc='r')
npslam.solveTree()
npslam.redrawAll()


for i in range(1,11):
    print ('x'+str(i)+'='), npslam.getParticles('x'+str(i)).mean(axis=1).T

v11 = npslam.addOdo([10.0, 0, np.pi/2])
v12 = npslam.addOdo([15.0, 0, 0])
v13 = npslam.addOdo([15.0, 0, np.pi/2])
v14 = npslam.addOdo([40.0, 0, np.pi/2])
v15 = npslam.addOdo([30.0, 0, np.pi/2])
v16 = npslam.addOdo([20.0, 0, 0])
v17 = npslam.addOdo([10.0, 0, 0])
npslam.addLandmBR([-np.pi/2,5.0], npslam.getID('x17'), to=l1)
npslam.drawFactorGraphpdf()
poses,landms = npslam.ls()


print npslam.ls()
npslam.redrawAll(pname='initp',posec='b',lname='landminit',landmc='r')
npslam.solveTree()
npslam.redrawAll()



npslam.sendCmd("QUIT")
npslam.disconnectServer()

# Vb = npslam.getParticles('x3')
# npslam.drawParticles2d([Vb], name='x3init', lname='yawinit', lc=['b'])
# Vb = npslam.getParticles('l1')
# npslam.drawParticles2d([Vb], name='lminit', lname='yawinit', lc=['b'])

# Vl1 = npslam.getParticles('l1');
# npslam.drawParticles2d([Vl1], c=['r'], name='landm')


# Vx1 = npslam.getParticles('x1');
# Vx2 = npslam.getParticles('x2');
# Vx3 = npslam.getParticles('x3');
# Vx4 = npslam.getParticles('x4');
# Vx5 = npslam.getParticles('x5');
# Vx6 = npslam.getParticles('x6');
# npslam.drawParticles2d([Vx1,Vx2,Vx3, Vx4], c=['g','g','g','g'], name='poses')
