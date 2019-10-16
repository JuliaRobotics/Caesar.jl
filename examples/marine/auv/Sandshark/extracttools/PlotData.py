# plot stuff

import matplotlib.pyplot as plt
import os
import glob
import numpy as np

range_csv = '/home/dehann/data/sandshark/full_wombat_2018_07_09/extracted/matchedfilter/particles/'
angle_csv = '/home/dehann/data/sandshark/full_wombat_2018_07_09/extracted/beamformer/particles/'

plt.ion()
fig, ax = plt.subplots()
x,y = [],[]
# sc = ax.scatter(x,y)
[n,X, V]=ax.hist(range(100),bins=50,normed=True)
# plt.xlim(0,2*np.pi)
# plt.ylim(0,np.pi)

for filename in sorted(glob.glob(angle_csv+"*.csv")):
    file = os.path.basename(filename)
    print file
    range_particles_file = range_csv + file
    angle_particles_file = angle_csv + file
    range_particles = np.genfromtxt(range_particles_file,delimiter=',')
    angle_particles = np.genfromtxt(angle_particles_file,delimiter=',')
    # azims = angle_particles[:,0]
    # incls = angle_particles[:,1]
    # sc.set_offsets(angle_particles)
    ax.cla()
    [n,X, V]=ax.hist(angle_particles[:,0],bins=50,normed=True)
    fig.canvas.draw_idle()
    plt.pause(0.1)
