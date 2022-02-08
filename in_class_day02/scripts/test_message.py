#!/usr/bin/env python3
""" This script explores publishing ROS messages in ROS using Python. """
from typing import *
import rospy
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header

class TestSendNode:
	publisher: rospy.Publisher

	def __init__(self, name='test_message', topic='/my_point'):
		rospy.init_node('test_message')
		self.publisher = rospy.Publisher(topic, PointStamped, queue_size=10)
	
	def make_point(self) -> PointStamped:
		my_header = Header(stamp=rospy.Time.now(), frame_id='odom')
		my_point = Point(2.0, 2.0, 0.0)
		my_point_stamped = PointStamped(header=my_header, point=my_point)
		return my_point_stamped
	
	def run(self):
		r = rospy.Rate(2)
		while not rospy.is_shutdown():
			self.publisher.publish(self.make_point())
			r.sleep()

if __name__ == "__main__":
	node = TestSendNode()
	node.run()
