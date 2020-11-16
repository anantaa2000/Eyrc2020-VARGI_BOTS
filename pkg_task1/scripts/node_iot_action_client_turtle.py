#!/usr/bin/env python

import rospy
import actionlib
import time
import requests

from pkg_task1.msg import msgTurtleAction
from pkg_task1.msg import msgTurtleGoal
from pkg_task1.msg import msgTurtleResult

from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal
from pkg_ros_iot_bridge.msg import msgRosIotResult

from pkg_ros_iot_bridge.msg import msgMqttSub

position = [0, 0, 0]

class ActionClientTurtle:

	def __init__(self):
		self.ac = actionlib.ActionClient('/action_ros_iot',msgRosIotAction)
		self._goal_handles = {}

		param_config_iot = rospy.get_param('config_iot')
		self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
		self._config_mqtt_sub_topic = param_config_iot['mqtt']['sub_cb_ros_topic']

		self.parameter = {}
		self.ac.wait_for_server()
		rospy.loginfo("Action server up")
		self._handle_ros_sub = rospy.Subscriber(self._config_mqtt_sub_topic, msgMqttSub, self.callback)
		self.sac = actionlib.SimpleActionClient('/action_turtle', msgTurtleAction)
		self.sac.wait_for_server()
		rospy.loginfo("Action client up , to send goal")


	def proceed(self, d, s, h):
		global position
		self.send_goal(d,s)
		rospy.sleep(10)
		goal_handle = self.send_goal_iot('mqtt', "pub", self._config_mqtt_pub_topic, position )
		self._goal_handles['h'] = goal_handle


	def callback(self, data):
		command =''
		command = str(data.message)
		if (command == 'start'):
			self.proceed(2,0,1)
			self.proceed(2,60,2)
			self.proceed(2,60,3)
			self.proceed(2,60,4)
			self.proceed(2,60,5)
			self.proceed(2,60,6)




	def on_transition(self, goal_handle):
		result = msgRosIotResult()
		index = 0
		for i in self._goal_handles:
			if self._goal_handles[i] == goal_handle:
				index = 1
				break

	def send_goal_iot(self, arg_protocol, arg_mode, arg_topic, arg_message):
		goal = msgRosIotGoal()
		goal.protocol = arg_protocol
		goal.mode = arg_mode
		goal.topic = arg_topic
		goal.message = arg_message

		goal_handle = self.ac.send_goal(goal, self.on_transition, None)
		return goal_handle


	def send_goal(self, arg_dis ,arg_angle):
		goal = msgTurtleGoal(distance=arg_dis, angle=arg_angle)
		self.sac.send_goal(goal, done_cb =self.done_callback)


	def done_callback(self, result, status):
		global position
		position[0]= status.final_x
		position[1] =status.final_y
		position[2]= status.final_theta

def main():

	rospy.init_node('node_iot_action_client_turtle')
	obj_client = ActionClientTurtle()
	rospy.spin()


if __name__=='__main__':
	main()
