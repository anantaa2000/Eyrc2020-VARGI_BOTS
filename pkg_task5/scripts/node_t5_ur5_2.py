#! /usr/bin/env python
'''NODE T5 UR5_2 USED TO CONTROL UR5_2 ARM
AND SORT PACKAGES DISPATCHED BY UR5_1 ARM USING THE 
COLOR PUBLISHED BY UR5_1 NODE WHILE DISPATCHING THE PKG
WITH THE HELP OD LOGICAL CAM 2 TO DETECT THE PKG AND 
STOPPING THE CONVEYOR SO  THAT THE ARM CAN 
REACH THE PKG AND SORT IT IN COROSPONDING BINS.'''

import math
import time
import datetime
import sys
import json
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import yaml
from std_srvs.srv import Empty
from pkg_vb_sim.srv import vacuumGripper  #INCLUDING SERVICE FILES
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse
from pkg_vb_sim.msg import ConveyorBeltState
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_task5.msg import packageColor
from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal
from pkg_ros_iot_bridge.msg import msgRosIotResult
from pkg_ros_iot_bridge.msg import msgMqttSub


class Ur5Moveit():
    '''CLASS DECLAERATION CONTAINING ALL CLASS
        METHODS AND FUNCTIONS.'''

    # Constructor
    def __init__(self, arg_robot_name):

        rospy.init_node('node_t5_ur5_2', anonymous=True)

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"

        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['sub_cb_ros_topic']


        self._commander = moveit_commander.roscpp_initialize(sys.argv)

        self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns +
        	                                             "/robot_description", ns=self._robot_ns)

        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)

        self._group = moveit_commander.MoveGroupCommander(self._planning_group,
        	                                                 robot_description=self._robot_ns +
        	                                                 "/robot_description",
        	                                                 ns=self._robot_ns)

        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns +
        	                                                    '/move_group/display_planned_path',
        	                                                    moveit_msgs.msg.DisplayTrajectory,
        	                                                    queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
        	   self._robot_ns +
        	   '/execute_trajectory',
        	   moveit_msgs.msg.ExecuteTrajectoryAction)

        self.ac = actionlib.ActionClient('/action_ros_iot', msgRosIotAction)
        self._exectute_trajectory_client.wait_for_server()
        self.ac.wait_for_server()

        self._activate_vacuum_gripper = rospy.ServiceProxy(
        	   '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2',
        	   vacuumGripper)

        self._activate_ConveyorBelt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',
        	                                                conveyorBeltPowerMsg)

        self.conveyor_state = rospy.Subscriber('/gazebo_sim/conveyor/state',
        	                                      ConveyorBeltState, self.conveyor_state_callback)


        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''


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
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self._file_path))


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

        self.red_bin = [math.radians(69),
                        math.radians(-41),
                        math.radians(80),
                        math.radians(-131),
                        math.radians(-91),
                        math.radians(-1)]

        self.yellow_bin = [math.radians(-3),
                           math.radians(-41),
                           math.radians(80),
                           math.radians(-131),
                           math.radians(-91),
                           math.radians(-1)]

        self.green_bin = [math.radians(-93),
                          math.radians(-41),
                          math.radians(80),
                          math.radians(-131),
                          math.radians(-91),
                          math.radians(-1)]

        self.home_pose = [math.radians(172.158644502),
                          math.radians(-40.0489379456),
                          math.radians(58.2512942738),
                          math.radians(-108.158242078),
                          math.radians(-90.0037747571),
                          math.radians(-7.86462864917)]

        self.package_Color = rospy.Subscriber("/topic_package_Color",
        	                                     packageColor, self.callback_color)
        self.color_list = []



    def conveyor_state_callback(self, state):
        '''FUNCTION TO CHECK CONVEYOR STATE
        USED BY CALLBACK FUNCTION.'''
        self.conveyor_power = state.power

    def callback_color(self, color_msg):
        '''CALLBACK COLOR FUNCTION
        USED TO SAVE INCOMMING COLOR MESSAGES PUBLISHED BY THE UR5_1 NODE WHILE DISPATCHING
        THE ORDER/PKG.'''
        print color_msg.color
        self.color_list.append(color_msg.color)
        print self.color_list


    def callback_logical_cam_2(self, msg):
        '''LOGICAL CAMERA 1 CALLBACK FUNCTION
        TO CHECK AVILABLE PACKAGE UNDER CAMERA
        AND TO SORT AVILABLE PKGS ACCORDIN TO INCOMING COLOR '''
        if msg.models:
            if msg.models[0].type == 'ur5':
                if self.conveyor_power == 0:
                    self.conveyor(100)
                if len(msg.models) >= 2:
                    if msg.models[1].pose.position.y <= 0.10:
                        self.conveyor(0)
                        print self.color_list

                        if  self.color_list[0][0] == "yellow":
                            data = json.loads(self.color_list[0][1])
                            print data
                            rospy.sleep(0.2)
                            self.attach_box()
                            self.moveit_hard_play_planned_path_from_file(self._file_path,
                            	                                            'yellow_bin.yaml', 5)
                            self.detach_box()
                            spread = ["OrderShipped", data[1], data[2], data[3], data[4], data[5],
                                      data[6], data[7], self.get_time_str(),
                                      self.get_Estimated_Delivery_date(3)]
                            goal_handle = self.send_goal_iot('mqtt', "pub",
                            	                                self._config_mqtt_pub_topic, spread)

                            del self.color_list[0]

                        if self.color_list[0][0] == "green":
                            data = json.loads(self.color_list[0][1])
                            print data
                            rospy.sleep(0.2)
                            self.attach_box()
                            self.moveit_hard_play_planned_path_from_file(self._file_path,
                            	                                            'green_bin.yaml', 5)
                            self.detach_box()
                            spread = ["OrderShipped", data[1], data[2], data[3], data[4], data[5],
                                      data[6], data[7], self.get_time_str(),
                                      self.get_Estimated_Delivery_date(5)]
                            goal_handle = self.send_goal_iot('mqtt', "pub",
                            	                                self._config_mqtt_pub_topic, spread)
                            del self.color_list[0]

                        if self.color_list[0][0] == "red":
                            data = json.loads(self.color_list[0][1])
                            print data
                            rospy.sleep(0.2)
                            self.attach_box()
                            self.moveit_hard_play_planned_path_from_file(self._file_path,
                            	                                            'red_bin.yaml', 5)
                            self.detach_box()
                            spread = ["OrderShipped", data[1], data[2], data[3], data[4], data[5],
                                      data[6], data[7], self.get_time_str(),
                                      self.get_Estimated_Delivery_date(1)]
                            goal_handle = self.send_goal_iot('mqtt', "pub",
                            	                                self._config_mqtt_pub_topic, spread)
                            del self.color_list[0]

            if msg.models[0].type != 'ur5':
                if msg.models[0].pose.position.y <= 0.10:
                    self.conveyor(0)
                    self.hard_set_joint_angles(self.home_pose, 5)

        else:
            self.hard_set_joint_angles(self.home_pose, 5)




    def clear_octomap(self):
        '''CLEARS OCTOMAP OF SENSORS
        docstrings and does nothing really.'''
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()



    def conveyor(self, power):
        '''CONVEYOR FUNCTION
        USED TO START/STOP CONVEYOR.'''
        req = conveyorBeltPowerMsgRequest()
        req.power = power
        response = self._activate_ConveyorBelt(req)
        print response


    def attach_box(self):
        '''ATTACH BOX
        FUNCTION USED TO ATTACH BOX TO UR5 ARM IN  GAZEBO AND RVIZ BOTH.'''
        self.box_pose = geometry_msgs.msg.PoseStamped()
        self.box_pose.header.frame_id = "ur5_wrist_3_link"
        self.box_pose.pose.orientation.w = 1.0
        self.box_pose.pose.position.x = 0.0
        self.box_pose.pose.position.y = 0.2
        self.box_pose.pose.position.z = 0.0
        self.box_name = "box"
        self._scene.add_box(self.box_name, self.box_pose, size=(0.15, 0.15, 0.15))
        touch_links = "ur5_wrist_3_link"                         #self._eef_link
        self._scene.attach_box(self._eef_link, self.box_name, touch_links=touch_links)
        req_attach = vacuumGripperRequest()
        req_attach.activate_vacuum_gripper = True
        resp = self._activate_vacuum_gripper(req_attach)
        print resp



    def detach_box(self):
        '''DETACH BOX
        USED TO DETACH BOX FROM UR5 IN BOTH GAZEBO AND RVIZ.'''
        self._scene.remove_attached_object(self._eef_link, name=self.box_name)
        req_detach = vacuumGripperRequest()
        req_detach.activate_vacuum_gripper = False
        resp = self._activate_vacuum_gripper(req_detach)
        print resp
        self._scene.remove_world_object(self.box_name)



    def set_joint_angles(self, arg_list_joint_angles):
        '''SET JOINT ANGLES FUNCTION
        USED TO SET JOINTS ANGLES.'''

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)
        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        '''HARD SET JOINT ANGLES
        FUNCTION TO FORCEFULLY SET JOINTS ANGLES.'''

        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and  (flag_success is False)):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))
            # self.clear_octomap()


    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        '''MOVEIT PLAY PLANNED PATH FILE
        USED TO PLAY TRAJECTORY SAVED IN YAML FILE.'''
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)

        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret


    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        '''HARD MOVEIT PLAY PLANNED PATH FILE
        USED TO FORCEFULLY PLAY PATH FRON YAML FILE.'''
        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))
            # self.clear_octomap()

        return True



    def send_goal_iot(self, arg_protocol, arg_mode, arg_topic, arg_message):
        '''SEND GOAL IOT
        FUNCTION USED TO PUBLISH MESSAGE ON MQTT TOPIC.'''
        goal = msgRosIotGoal()
        goal.protocol = arg_protocol
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.message = arg_message

        goal_handle = self.ac.send_goal(goal, self.on_transition, None)
        return goal_handle



    def on_transition(self, goal_handle):
        '''ON TRANSITON FUNCTION
        USED BY SEND GOAL IOT AS CALLBACK.'''
        result = msgRosIotResult()
        index = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
                index = 1
                break


    def get_time_str(self):
        '''GET TIME STR FUNCTION
        FUNCTION IS USED TO GET SHIPPED TIME IN STRING FORMAT.'''
        timestamp = int(time.time())
        value = datetime.datetime.fromtimestamp(timestamp)
        str_time = value.strftime('%Y-%m-%d %H:%M:%S')

        return str_time

    def get_Estimated_Delivery_date(self, days):
        '''GET TIME STR FUNCTION
        FUNCTION IS USED TO GET DELIVERY TIME IN STRING FORMAT.'''
        date = datetime.datetime.today() + datetime.timedelta(days)
        return date.strftime("%x")


    # Destructor

    def __del__(self):
        '''DESTRUCTOR
        TO DELETE CLASS OBJECT .'''
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():
    '''MAIN FINCTION
    CLASS OBJECT IS CREATED FOR UR5_2 ARM AND LOGICAL CAM2 SUBSCRIBER IS CALLED.'''

    ur5 = Ur5Moveit("ur5_2")
    logical_camera_2 = rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage,
    	                                   ur5.callback_logical_cam_2, queue_size=1)

    rospy.spin()

    del ur5



if __name__ == '__main__':
    main()
