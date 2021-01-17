#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image


import yaml
import os
import math
import time
import sys
import copy
import tf2_ros
import tf2_msgs.msg

from std_srvs.srv import Empty
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from pkg_vb_sim.srv import vacuumGripper  #INCLUDING SERVICE FILES
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse
from pkg_vb_sim.msg import ConveyorBeltState

from pkg_vb_sim.msg import LogicalCameraImage
from pkg_task4.msg import packageColor


class Ur5Moveit:

	# Constructor
	def __init__(self, arg_robot_name):

		rospy.init_node('node_t4_ur5_1', anonymous=True)

		self._robot_ns = '/'  + arg_robot_name
		self._planning_group = "manipulator"
		
		self._commander = moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
		self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
		self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
		self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
		self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
		self._activate_vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
		self._activate_ConveyorBelt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
		self.conveyor_state = rospy.Subscriber('/gazebo_sim/conveyor/state',ConveyorBeltState,self.conveyor_state_callback)

		self.logical_camera_1 = rospy.Subscriber("/eyrc/vb/logical_camera_1", LogicalCameraImage, self.callback_logical_cam_1, queue_size = 1)
		self._exectute_trajectory_client.wait_for_server()
		self._planning_frame = self._group.get_planning_frame()
		self._eef_link = self._group.get_end_effector_link()
		self._group_names = self._robot.get_group_names()
		self._box_name = ''
		self.bridge = CvBridge()
		self.inventory = []


		# Attribute to store computed trajectory by the planner	
		self._computed_plan = ''

		# Current State of the Robot is needed to add box to planning scene
		self._curr_state = self._robot.get_current_state()

		rospy.loginfo(
			'\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
		rospy.loginfo(
			'\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
		rospy.loginfo(
			'\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


		rp = rospkg.RosPack()
		self._pkg_path = rp.get_path('pkg_task4')
		self._file_path = self._pkg_path + '/config/saved_trajectories/'
		rospy.loginfo( "Package Path: {}".format(self._file_path) )


		rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

		self.packagen_00 = [math.radians(162),math.radians(-115),math.radians(8),math.radians(-90),math.radians(19),math.radians(0)]

		self.packagen_01 = [math.radians(130),math.radians(-89),math.radians(-29),math.radians(-69),math.radians(46),math.radians(0)]

		self.packagen_02 = [math.radians(57),math.radians(-109),math.radians(-7),math.radians(-67),math.radians(119),math.radians(0)]

		self.packagen_10 = [math.radians(-52),math.radians(-95),math.radians(83),math.radians(-171),math.radians(-128),math.radians(0)]

		self.packagen_11 = [math.radians(125),math.radians(-74),math.radians(-64),math.radians(136),math.radians(-57),math.radians(0)]

		self.packagen_12 = [math.radians(60),math.radians(-91),math.radians(-49),math.radians(143),math.radians(-105),math.radians(0)]

		self.packagen_20 = [math.radians(-51),math.radians(-97),math.radians(95),math.radians(1),math.radians(128),math.radians(0)]

		self.packagen_21 = [math.radians(122),math.radians(-60),math.radians(-105),math.radians(167),math.radians(-59),math.radians(0)]

		self.packagen_22 = [math.radians(53),math.radians(-82),math.radians(-92),math.radians(176),math.radians(-127),math.radians(0)]

		self.packagen_30 = [math.radians(-51),math.radians(-89),math.radians(116),math.radians(-30),math.radians(129),math.radians(0)]

		self.packagen_31 = [math.radians(-117),math.radians(-118),math.radians(135),math.radians(-19),math.radians(62),math.radians(0)]

		self.packagen_32 = [math.radians(-163),math.radians(-97),math.radians(122),math.radians(-30),math.radians(17),math.radians(0)]

		self.package_drop = [math.radians(173),math.radians(-42),math.radians(56),math.radians(-104),math.radians(-90),math.radians(-7)]

		self.straightUp = [math.radians(180),math.radians(-90),math.radians(0),math.radians(-180),math.radians(-90),math.radians(0)]




	def conveyor_state_callback(self, state):
		self.conveyor_power = state.power


	def callback_logical_cam_1(self, msg):
		if(msg.models):
			if(self.conveyor_power == 0):
				self.conveyor(100)


	def conveyor(self, power):
		req = conveyorBeltPowerMsgRequest()
		req.power = power
		response = self._activate_ConveyorBelt(req)
		print response


	def inventory_callback(self, raw_image):
		self.inventory =self.image_conversion(raw_image)
		print(self.inventory)

	def clear_octomap(self):
		clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
		return clear_octomap_service_proxy()

	def set_joint_angles(self, arg_list_joint_angles):
		self._group.set_joint_value_target(arg_list_joint_angles)
		self._computed_plan = self._group.plan()
		flag_plan = self._group.go(wait=True)
		return flag_plan

	def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

		number_attempts = 0
		flag_success = False
		
		while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
			number_attempts += 1
			flag_success = self.set_joint_angles(arg_list_joint_angles)
			rospy.logwarn("attempts: {}".format(number_attempts) )
			# self.clear_octomap()


	def get_qr_data(self,arg_image):
		qr_result = decode(arg_image)
		polygon=[]
		final_sorted = []
		for x in range(0,len(qr_result)):
			polygon.append(copy.deepcopy([qr_result[x].data, round(qr_result[x].rect[1]/100)*100, round(qr_result[x].rect[0]/100)*100 ]))  

		#NORMALIZING ALL VALUES FOR CORDINATES
		for ind in range (0,len(polygon)):
			if (polygon[ind][1] == 300.0):
				polygon[ind][1] = 0

			if (polygon[ind][1] == 400.0):
				polygon[ind][1] = 1

			if (polygon[ind][1] == 600.0):
				polygon[ind][1] = 2

			if (polygon[ind][1] == 700.0):
				polygon[ind][1] = 3

			if (polygon[ind][2] ==100 ):
				polygon[ind][2] = 0

			if (polygon[ind][2] == 300):
				polygon[ind][2] = 1

			if (polygon[ind][2] == 500 ):
				polygon[ind][2] = 2

		final_sorted = sorted(polygon,key=lambda k: (k[1],k[2]) )
		return(final_sorted)  

	

	def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
		file_path = arg_file_path + arg_file_name
		
		with open(file_path, 'r') as file_open:
			loaded_plan = yaml.load(file_open)
		
		ret = self._group.execute(loaded_plan)
		# rospy.logerr(ret)
		return ret

	
	def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
		number_attempts = 0
		flag_success = False

		while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
			number_attempts += 1
			flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
			rospy.logwarn("attempts: {}".format(number_attempts) )
			# self.clear_octomap()
		
		return True


	def image_conversion(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"mono8")
		except CvBridgeError as e:
			rospy.logerr(e)


		th3 = cv2.adaptiveThreshold(cv_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
			cv2.THRESH_BINARY,15,6)



		shelf_info_relative_position = self.get_qr_data(th3)
		return(shelf_info_relative_position)


	def attach_box(self, timeout=4):
		self.box_pose = geometry_msgs.msg.PoseStamped()
		self.box_pose.header.frame_id = "ur5_wrist_3_link"
		self.box_pose.pose.orientation.w = 1.0
		self.box_pose.pose.position.x = 0.0
		self.box_pose.pose.position.y = 0.2
		self.box_pose.pose.position.z = 0
		self.box_name = "box"
		self._scene.add_box(self.box_name, self.box_pose, size=(0.16, 0.16, 0.16))
		touch_links = "ur5_wrist_3_link"						 #self._eef_link
		self._scene.attach_box(self._eef_link, self.box_name, touch_links=touch_links)
		req_attach = vacuumGripperRequest()
		req_attach.activate_vacuum_gripper = True
		resp = self._activate_vacuum_gripper(req_attach)
		print(resp)



	def detach_box(self, timeout=4):
		self._scene.remove_attached_object(self._eef_link, name=self.box_name)
		req_detach = vacuumGripperRequest()
		req_detach.activate_vacuum_gripper = False
		resp = self._activate_vacuum_gripper(req_detach)
		print(resp)
		self._scene.remove_world_object(self.box_name)



	# Destructor

	def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo(
			'\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():
	color = rospy.Publisher('/package_Color', packageColor, queue_size=20)
	ur5 = Ur5Moveit("ur5_1")
	rospy.sleep(5)

	msg= rospy.wait_for_message("/eyrc/vb/camera_1/image_raw", Image)
	qr_data_list = ur5.image_conversion(msg)
	print(qr_data_list)

	count =0
	
	for x in range(0, len(qr_data_list)):
		
		if( count<9):

			if(qr_data_list[x][1] == 0 and qr_data_list[x][2]==0):
				count+=1
				# ur5.hard_set_joint_angles(ur5.packagen_00,5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg00.yaml', 5)
				ur5.attach_box()
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg00_to_drop.yaml', 5)
				ur5.detach_box()
				color.publish(qr_data_list[x][0])
				# ur5.hard_set_joint_angles(ur5.straightUp, 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_up.yaml', 5)
				


			if(qr_data_list[x][1] == 0 and qr_data_list[x][2]==1):
				count+=1
				# ur5.hard_set_joint_angles(ur5.packagen_01,5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg01.yaml', 5)
				ur5.attach_box()
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg01_to_drop.yaml', 5)
				ur5.detach_box()
				color.publish(qr_data_list[x][0])

				# ur5.hard_set_joint_angles(ur5.straightUp, 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_up.yaml', 5)


			if(qr_data_list[x][1] == 0 and qr_data_list[x][2]==2):
				count+=1
				# ur5.hard_set_joint_angles(ur5.packagen_02,5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg02.yaml', 5)
				ur5.attach_box()
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg02_to_drop.yaml', 5)
				ur5.detach_box()
				color.publish(qr_data_list[x][0])
				# ur5.hard_set_joint_angles(ur5.straightUp, 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_up.yaml', 5)
				


			if(qr_data_list[x][1] == 1 and qr_data_list[x][2]==0):
				count+=1  
				# ur5.hard_set_joint_angles(ur5.packagen_10,5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg10.yaml', 5)
				ur5.attach_box()
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg10_to_drop.yaml', 5)
				ur5.detach_box()
				color.publish(qr_data_list[x][0])
				# ur5.hard_set_joint_angles(ur5.straightUp, 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_up.yaml', 5)
				


			if(qr_data_list[x][1] == 1 and qr_data_list[x][2]==1):
				count+=1 
				# ur5.hard_set_joint_angles(ur5.packagen_11,5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg11.yaml', 5)
				ur5.attach_box()
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg11_to_drop.yaml', 5)
				ur5.detach_box()
				color.publish(qr_data_list[x][0])
				ur5.hard_set_joint_angles(ur5.straightUp, 5)
				


			if(qr_data_list[x][1] == 1 and qr_data_list[x][2]==2):
				count+=1  
				# ur5.hard_set_joint_angles(ur5.packagen_12,5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg12.yaml', 5)
				ur5.attach_box()
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg12_to_drop.yaml', 5)
				ur5.detach_box()
				color.publish(qr_data_list[x][0])
				# ur5.hard_set_joint_angles(ur5.straightUp, 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_up.yaml', 5)


			if(qr_data_list[x][1] == 2 and qr_data_list[x][2]==0):
				count+=1 
				# ur5.hard_set_joint_angles(ur5.packagen_20,5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg20.yaml', 5)
				ur5.attach_box()
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg20_to_drop.yaml', 5)
				ur5.detach_box()
				color.publish(qr_data_list[x][0])
				# ur5.hard_set_joint_angles(ur5.straightUp, 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_up.yaml', 5)


			if(qr_data_list[x][1] == 2 and qr_data_list[x][2]==1):
				count+=1  
				# ur5.hard_set_joint_angles(ur5.packagen_21,5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg21.yaml', 5)
				ur5.attach_box()
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg21_to_drop.yaml', 5)
				ur5.detach_box()
				color.publish(qr_data_list[x][0])
				# ur5.hard_set_joint_angles(ur5.straightUp, 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_up.yaml', 5)


			if(qr_data_list[x][1] == 2 and qr_data_list[x][2]==2):
				count+=1
				# ur5.hard_set_joint_angles(ur5.packagen_22,5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg22.yaml', 5)
				ur5.attach_box()
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg22_to_drop.yaml', 5)
				ur5.detach_box()
				color.publish(qr_data_list[x][0])
				# ur5.hard_set_joint_angles(ur5.straightUp, 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_up.yaml', 5)


			if(qr_data_list[x][1] == 3 and qr_data_list[x][2]==0):
				count+=1  
				# ur5.hard_set_joint_angles(ur5.packagen_30,5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg30.yaml', 5)
				ur5.attach_box()
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg30_to_drop.yaml',5)
				ur5.detach_box()
				color.publish(qr_data_list[x][0])
				# ur5.hard_set_joint_angles(ur5.straightUp, 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_up.yaml', 5)


			if(qr_data_list[x][1] == 3 and qr_data_list[x][2]==1):
				count+=1  
				# ur5.hard_set_joint_angles(ur5.packagen_31,5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg31.yaml', 5)
				ur5.attach_box()
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg31_to_inter31.yaml', 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'inter31_to_drop.yaml', 5)
				ur5.detach_box()
				color.publish(qr_data_list[x][0])
				# ur5.hard_set_joint_angles(ur5.straightUp, 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_up.yaml', 5)


			if(qr_data_list[x][1] == 3 and qr_data_list[x][2]==2):
				count+=1  
				# ur5.hard_set_joint_angles(ur5.packagen_32,5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg32.yaml', 5)
				ur5.attach_box()
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg32_to_drop.yaml', 5)
				ur5.detach_box()
				color.publish(qr_data_list[x][0])
				# ur5.hard_set_joint_angles(ur5.straightUp, 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_up.yaml', 5)


	rospy.spin()

	del ur5



if __name__ == '__main__':
	main()


