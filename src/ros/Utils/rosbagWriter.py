
import rosbag
import sys

class RosbagWriter:
    def __init__(self, bag_file_name, topic_name):
        #bag_file_name = sys.argv[1]
        #topic_name = sys.argv[2]

        self.bag = rosbag.Bag(bag_file_name)
        self.bag_contents = self.bag.read_messages(topics=[topic_name])

        self.idx = 0
        self.topic_size = self.bag.get_message_count(topic_filters=topic_name)

    def write_message(self,channel,msg,header=msg.header.stamp):
        if (self.idx < self.topic_size):
            self.bag.write(channel, msg, header)
        else:
            self.bag.close()
            return False