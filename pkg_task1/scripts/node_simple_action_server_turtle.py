#!/usr/bin/env python

import rospy
import actionlib
import math
import time

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from pkg_task1.msg import msgTurtleAction
from pkg_task1.msg import msgTurtleGoal
from pkg_task1.msg import msgTurtleResult
from pkg_task1.msg import msgTurtleFeedback

class SimpleActionServerTurtle:

	def __init__(self):

		self.sas = actionlib.SimpleActionServer('/action_turtle', msgTurtleAction, execute_cb=self.func_on_rx_goal, auto_start=False)

		self._config_ros_pub_topic = '/turtle1/cmd_vel'
		self._config_ros_sub_topic = '/turtle1/pose'

		self._curr_x = 0
		self._curr_y = 0
		self._curr_theta = 0

		self.sas.start()
		rospy.loginfo("Started Turtle Simple Action Server.")

	def func_ros_sub_callback(self, pose_message):

		self._curr_x = pose_message.x
		self._curr_y = pose_message.y
		self._curr_theta = pose_message.theta

	def func_move_straight(self, param_dis, param_speed, param_dir):

	    obj_vel_mssg = Twist()
	    obj_pose_mssg = Pose()

	    start_x = self._curr_x
	    start_y = self._curr_y

	    handle_pub_vel = rospy.Publisher(self._config_ros_pub_topic, Twist, queue_size =10)

	    var_loop_rate = rospy.Rate(10)

	    if(param_dir == 'b'):
	    	obj_vel_mssg.linear.x = (-1) * abs(int(param_speed))
	    else:
	    	obj_vel_mssg.linear.x = abs(int(param_speed))

	    dis_moved = 0.0

	    while not rospy.is_shutdown():


	    	if ((dis_moved < param_dis)):
	    		handle_pub_vel.publish(obj_vel_mssg)

	    		var_loop_rate.sleep()

	    		dis_moved = abs(math.sqrt(((self._curr_x - start_x) ** 2) +((self._curr_y - start_y) ** 2)))
	    	else:
	    		break

	    obj_vel_mssg.linear.x = 0
	    handle_pub_vel.publish(obj_vel_mssg)


	def func_rotate(self, param_degree , param_speed, param_dir):

		obj_vel_mssg = Twist()
		obj_pose_mssg = Pose()

		start_degree = abs(math.degrees(self._curr_theta))
		current_degree = abs(math.degrees(self._curr_theta))

		handle_pub_vel = rospy.Publisher(self._config_ros_pub_topic, Twist ,queue_size = 10)

		var_loop_rate = rospy.Rate(10)


		if(param_dir =='a'):
			obj_vel_mssg.angular.z = math.radians(abs(int(param_speed)))
		else:
			obj_vel_mssg.angular.z = (-1) * \
                math.radians(abs(int(param_speed)))

		degree_rotated = 0.0

		while not rospy.is_shutdown():
			if((round(degree_rotated)< param_degree)):

				handle_pub_vel.publish(obj_vel_mssg)

				var_loop_rate.sleep()

				current_degree = abs(math.degrees(self._curr_theta))
				degree_rotated = abs(current_degree - start_degree)
			else:
				break



		obj_vel_mssg.angular.z = 0
		handle_pub_vel.publish(obj_vel_mssg)

	def func_on_rx_goal(self, obj_msg_goal):
		flag_success = False
		flag_preempted = False

		self.func_rotate(obj_msg_goal.angle , '10', 'a')
		self.func_move_straight(obj_msg_goal.distance, '1', 'f')

		obj_msg_result = msgTurtleResult()
		obj_msg_result.final_x = self._curr_x
		obj_msg_result.final_y = self._curr_y
		obj_msg_result.final_theta = self._curr_theta

		self.sas.set_succeeded(obj_msg_result)


def main():
	rospy.init_node('node_simple_action_server_turtle')

	obj_server = SimpleActionServerTurtle()

	handle_sub_pose = rospy.Subscriber(obj_server._config_ros_sub_topic, Pose, obj_server.func_ros_sub_callback)

	rospy.spin()



if __name__ == '__main__':
	main()
