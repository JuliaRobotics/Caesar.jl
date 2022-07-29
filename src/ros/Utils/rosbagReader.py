#

# written by kurransingh

import rosbag
import sys

class RosbagParser:
    def __init__(self, bag_file_name, topic_name): #, compression=None):
        #bag_file_name = sys.argv[1]
        #topic_name = sys.argv[2]

        self.bag = rosbag.Bag(bag_file_name, 'r') #, compression=compression)
        self.bag_contents = self.bag.read_messages(topics=[topic_name])

        self.idx = 0
        self.topic_size = self.bag.get_message_count(topic_filters=topic_name)

    def get_next_message(self):
        if (self.idx < self.topic_size):
            topic, msg, time = next(self.bag_contents)
            # https://stackoverflow.com/questions/18547878/attribute-error-next
            # topic, msg, time = self.bag_contents.__next__()
            self.idx += 1
            return topic, msg, time
        else:
            self.bag.close()
            return False, False, False
