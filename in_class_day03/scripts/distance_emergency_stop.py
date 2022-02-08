#!/usr/bin/env python3
""" Drive forward until the Neato hits something. """
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import LaserScan
import math
import rospy

"""
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
"""

class DistanceEmergencyStopNode:
	def __init__(self, name='distance_emergency_stop'):
		rospy.init_node(name)
		rospy.Subscriber('/stable_scan', LaserScan, self.process_scan_data)
		self.target = rospy.get_param('~target', 2)
		self.speed = rospy.get_param('~speed', 1)
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	def process_scan_data(self, msg):
		relevant_data = msg.ranges[-45:-1] + msg.ranges[:45]
		mean_distance = sum(relevant_data) / len(relevant_data)
		speed = (mean_distance - self.target) * self.speed
		speed = min(1, speed)
		print(f"Target: {self.target}, mean distance: {mean_distance}, speed: {speed}")
		self.set_speed(speed, 0)
	
	def set_speed(self, forward, angular):
		twist = Twist(
			linear=Vector3(forward, 0, 0),
			angular=Vector3(0, 0, angular)
		)
		self.cmd_vel.publish(twist)

	def run(self):
		rospy.spin()

if __name__ == '__main__':
	node = DistanceEmergencyStopNode()
	node.run()