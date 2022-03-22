#!/usr/bin/env python3
""" Investigate receiving a message using a callback function """
from geometry_msgs.msg import PointStamped
import rospy


class TestReceiveNode:
    def __init__(self, name='receive_message', topic='/my_point'):
        rospy.init_node(name)
        rospy.Subscriber(topic, PointStamped, self.process_point)

    def process_point(self, msg):
        print(msg.header)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = TestReceiveNode()
    node.run()
