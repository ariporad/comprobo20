#!/usr/bin/env python3
""" Drive forward until the Neato hits something. """
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int8MultiArray
import rospy

class EmergencyStopNode:
	def __init__(self, name='emergency_stop'):
		rospy.init_node(name)
		rospy.Subscriber('/bump', Int8MultiArray, self.process_bump)
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	def process_bump(self, msg):
		did_bump = any(msg.data)
		self.set_speed(0 if did_bump else 0, 0)
	
	def set_speed(self, forward, angular):
		twist = Twist(
			linear=Vector3(forward, 0, 0),
			angular=Vector3(0, 0, angular)
		)
		self.cmd_vel.publish(twist)

	def run(self):
		rospy.spin()

if __name__ == '__main__':
	node = EmergencyStopNode()
	node.run()