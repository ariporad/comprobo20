#!/usr/bin/env python3
""" Drive forward until the Neato hits something. """
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import LaserScan
import math
import rospy
import time

class NodeState:
	node: 'StateMachineNode'

	def __init__(self, node: 'StateMachineNode'):
		self.node = node

	def process_scan_data(self, msg):
		""" Override Me! """
		pass
	
	def process_bump_data(self, msg):
		pass

	def activate(self):
		pass
	
	def deactivate(self):
		pass

class StateMachineNode:
	active_state: NodeState

	def __init__(self, starting_state: NodeState, name: str='state_machine'):
		rospy.init_node(name)
		self.active_state = starting_state(self)
		self.active_state.activate()
		rospy.Subscriber('/stable_scan', LaserScan, self.process_scan_data)
		rospy.Subscriber('/bump', Int8MultiArray, self.process_bump_data)
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	def transition(self, ToNodeType):
		print("Transitioning to:", ToNodeType.__name__)
		self.active_state.deactivate()
		self.active_state = ToNodeType(self)
		self.active_state.activate()

	def set_speed(self, forward, angular):
		twist = Twist(
			linear=Vector3(forward, 0, 0),
			angular=Vector3(0, 0, angular)
		)
		self.cmd_vel.publish(twist)

	def run(self):
		rospy.spin()

	def process_scan_data(self, msg):
		self.active_state.process_scan_data(msg)
	
	def process_bump_data(self, msg):
		self.active_state.process_bump_data(msg)


class MoveForwardState(NodeState):
	speed = 1

	def process_bump_data(self, msg):
		did_bump = any(msg.data)
		if did_bump:
			self.node.transition(MoveBackwardState)
		else:
			self.node.set_speed(self.speed, 0)

class MoveBackwardState(NodeState):
	distance_target = 1
	speed = 1

	def process_scan_data(self, msg):
		relevant_data = msg.ranges[-45:-1] + msg.ranges[:45]
		mean_distance = sum(relevant_data) / len(relevant_data)
		if mean_distance <= self.distance_target:
			self.node.set_speed(-1 * self.speed, 0)
		else:
			self.node.transition(RotatingLeftState)

class RotatingLeftState(NodeState):
	speed = 1

	def activate(self):
		self.node.set_speed(0, self.speed)
		rospy.sleep(1)
		self.node.transition(MoveForwardState)
	
if __name__ == '__main__':
	node = StateMachineNode(starting_state=MoveForwardState)
	node.run()