#!/usr/bin/python3

import os
import numpy as np

# dest will be made as a folder and several txt files will be saved within.
def storePyNNModelTxt(dest, mw):
  os.mkdir(dest)
  for i in range(len(mw)):
    np.savetxt(dest+'/w'+str(i)+'.txt',mw[i]) #,fmt='%f')
