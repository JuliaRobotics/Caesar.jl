
import rosbag
import sys

class RosbagWriter:
    def __init__(self, filename):
        self.filename = filename
        self.bag = rosbag.Bag(filename, 'w')

    def write_message(self, channel, msg):
        self.bag.write(channel, msg) # , header
    
    def close(self):
        self.bag.close()