#!/usr/bin/env python

# Author: Nick R. Rypkema (rypkema@mit.edu)
# License: MIT

import os
import sys
import re
import numpy as np

alogfile = str(sys.argv[1])
print alogfile
blogfile = alogfile[:-4] + 'blog'
print blogfile

num_rows = 8000
num_cols = 5
num_bytes = 4

filename_prefix = './daq_data/'
try:
    os.makedirs(filename_prefix)
except:
    pass

trigger_vals = []
publish_vals = []
daq_moos_times = []
daq_offsets = []
daq_nbytes = num_rows*num_cols*num_bytes
with open(alogfile,'r') as f:
    for line in f:
        if ("DAQ_BINARY_DATA" in line) and ("MOOS_BINARY" in line):
            data = str.split(line)
            binary = str.split(data[3],',')
            offset = str.split(binary[2],'=')[1]
            nbytes = re.split('=|<',binary[3])[1]
            if daq_nbytes != float(nbytes):
                sys.exit('number of bytes recorded by .alog file does not match specified data size in this script.')
            time = data[0]
            daq_moos_times.append(float(time))
            daq_offsets.append(int(offset))
        if  ("DAQ_TRIGGER_TIME" in line) and ('iMCC1608FS' in line) and ('STATUS' not in line):
            data = str.split(line)
            time = data[0]
            val = data[3]
            trigger_vals.append((float(time),int(float(val)*1e9)))
        if  ("DAQ_PUBLISH_TIME" in line) and ('iMCC1608FS' in line) and ('STATUS' not in line):
            data = str.split(line)
            time = data[0]
            val = data[3]
            publish_vals.append((float(time),int(float(val)*1e9)))

with open(blogfile,'rb') as f:
    for idx, offset in enumerate(daq_offsets):
        filename = filename_prefix + str(trigger_vals[idx][1]) + '.txt'
        f.seek(offset)
        daq_str_data = f.read(daq_nbytes)
        daq_data = np.frombuffer(daq_str_data, dtype=np.float32).reshape((num_rows, num_cols))
        np.savetxt(filename,daq_data,delimiter=',',fmt='%.6f')

if len(trigger_vals) > len(publish_vals):
    trigger_vals = trigger_vals[1:len(publish_vals)+1]
daq_timing_info = np.hstack([trigger_vals,publish_vals, np.array([daq_moos_times]).T])
np.savetxt('daq_trigger_publish_timing_info.txt',daq_timing_info,delimiter=',',fmt='%.3f',header='MOOS_TRIGGER_TIME,UTC_TRIGGER_TIME,MOOS_PUBLISH_TIME,UTC_PUBLISH_TIME,DAQ_MOOS_TIME',comments='')