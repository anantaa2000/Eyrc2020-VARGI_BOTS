#! /usr/bin/env python
'''NODE t5 UR5_1 USED TO CONTROL UR5_1 ARM
AND READ INCOMMING  ORDERS AND PROCESS THEM ACCORDING  TO THE INVENTORY 
AND  CONTROS 2D CAM WHICH DETECTS ALL PKGS IN SHELF WITH THEIR COLOR AND RELATIVE POSTION IN SHELF.'''

import math
import time
import datetime
import sys
import copy
import json
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image

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
import cv2


class Ur5Moveit:
    '''CLASS DECLAERATION CONTAINING ALL CLASS
        METHODS AND FUNCTIONS.'''

    # Constructor
    def __init__(self, arg_robot_name):
        '''CLASS INITIALIZATION
        INITIALIZE ALL VARIABLES.'''

        rospy.init_node('node_t5_ur5_1', anonymous=True)

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"

        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['sub_cb_ros_topic']

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns +
        	                                             "/robot_description",
        	                                             ns=self._robot_ns)

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

        self._activate_vacuum_gripper = rospy.ServiceProxy(
        	   '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',
        	   vacuumGripper)

        self._activate_ConveyorBelt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',
        	                                                conveyorBeltPowerMsg)

        self.conveyor_state = rospy.Subscriber('/gazebo_sim/conveyor/state', ConveyorBeltState,
        	                                      self.conveyor_state_callback)

        self.color = rospy.Publisher('/topic_package_Color', packageColor, queue_size=20)

        self.logical_camera_1 = rospy.Subscriber("/eyrc/vb/logical_camera_1",
        	                                        LogicalCameraImage,
        	                                        self.callback_logical_cam_1,
        	                                        queue_size=1)

        self.orders = rospy.Subscriber(self._config_mqtt_sub_topic, msgMqttSub,
        	                              self.callback_orders, queue_size=10)
        self.ac.wait_for_server()
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''
        self.bridge = CvBridge()
        self._goal_handles = {}
        self.red_pkgs = []
        self.yellow_pkgs = []
        self.green_pkgs = []
        self.orders_list = []

        self.packagen_00 = [math.radians(162),
                            math.radians(-115),
                            math.radians(8),
                            math.radians(-90),
                            math.radians(19),
                            math.radians(0)]

        self.packagen_01 = [math.radians(130),
                            math.radians(-89),
                            math.radians(-29),
                            math.radians(-69),
                            math.radians(46),
                            math.radians(0)]

        self.packagen_02 = [math.radians(57),
                            math.radians(-109),
                            math.radians(-7),
                            math.radians(-67),
                            math.radians(119),
                            math.radians(0)]

        self.packagen_10 = [math.radians(-52),
                            math.radians(-95),
                            math.radians(83),
                            math.radians(-171),
                            math.radians(-128),
                            math.radians(0)]

        self.packagen_11 = [math.radians(125),
                            math.radians(-74),
                            math.radians(-64),
                            math.radians(136),
                            math.radians(-57),
                            math.radians(0)]

        self.packagen_12 = [math.radians(60),
                            math.radians(-91),
                            math.radians(-49),
                            math.radians(143),
                            math.radians(-105),
                            math.radians(0)]

        self.packagen_20 = [math.radians(-51),
                            math.radians(-97),
                            math.radians(95),
                            math.radians(1),
                            math.radians(128),
                            math.radians(0)]

        self.packagen_21 = [math.radians(122),
                            math.radians(-60),
                            math.radians(-105),
                            math.radians(167),
                            math.radians(-59),
                            math.radians(0)]

        self.packagen_22 = [math.radians(53),
                            math.radians(-82),
                            math.radians(-92),
                            math.radians(176),
                            math.radians(-127),
                            math.radians(0)]

        self.packagen_30 = [math.radians(-51),
                            math.radians(-89),
                            math.radians(116),
                            math.radians(-30),
                            math.radians(129),
                            math.radians(0)]

        self.packagen_31 = [math.radians(-117),
                            math.radians(-118),
                            math.radians(135),
                            math.radians(-19),
                            math.radians(62),
                            math.radians(0)]

        self.packagen_32 = [math.radians(-163),
                            math.radians(-97),
                            math.radians(122),
                            math.radians(-30),
                            math.radians(17),
                            math.radians(0)]

        self.package_drop = [math.radians(173),
                             math.radians(-42),
                             math.radians(56),
                             math.radians(-104),
                             math.radians(-90),
                             math.radians(-7)]

        self.straightUp = [math.radians(180),
                           math.radians(-90),
                           math.radians(0),
                           math.radians(-180),
                           math.radians(-90),
                           math.radians(0)]


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




    def conveyor_state_callback(self, state):
        '''FUNCTION TO CHECK CONVEYOR STATE
        USED BY CALLBACK FUNCTION.'''
        self.conveyor_power = state.power


    def callback_logical_cam_1(self, msg):
        '''LOGICAL CAMERA 1 CALLBACK FUNCTION
        TO CHECK AVILABLE PACKAGE UNDER CAMERA.'''
        if msg.models:
            if self.conveyor_power == 0:
                self.conveyor(100)


    def conveyor(self, power):
        '''CONVEYOR FUNCTION
        USED TO START/STOP CONVEYOR.'''
        req = conveyorBeltPowerMsgRequest()
        req.power = power
        response = self._activate_ConveyorBelt(req)
        print response



    def clear_octomap(self):
        '''CLEARS OCTOMAP OF SENSORS
        docstrings and does nothing really.'''
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

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


    def get_qr_data(self, arg_image):
        '''GET QR DATA
        FUNCTION TO DECODE QR CODE PRESENT ON BOXES.'''
        qr_result = decode(arg_image)
        polygon = []
        final_sorted = []
        for x in range(0, len(qr_result)):
            polygon.append(copy.deepcopy([qr_result[x].data, round(qr_result[x].rect[1]/100)*100,
            	                             round(qr_result[x].rect[0]/100)*100]))

        #NORMALIZING ALL VALUES FOR CORDINATES
        for ind in range(0, len(polygon)):
            if polygon[ind][1] == 300.0:
                polygon[ind][1] = 0

            if polygon[ind][1] == 400.0:
                polygon[ind][1] = 1

            if polygon[ind][1] == 600.0:
                polygon[ind][1] = 2

            if polygon[ind][1] == 700.0:
                polygon[ind][1] = 3

            if polygon[ind][2] == 100:
                polygon[ind][2] = 0

            if polygon[ind][2] == 300:
                polygon[ind][2] = 1

            if polygon[ind][2] == 500:
                polygon[ind][2] = 2

        final_sorted = sorted(polygon, key=lambda k: (k[1], k[2]))
        return final_sorted


    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        '''MOVEIT PLAY PLANNED PATH FILE
        USED TO PLAY TRAJECTORY SAVED IN YAML FILE.'''
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)

        ret = self._group.execute(loaded_plan)
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
        return True


    def image_conversion(self, data):
        '''IMAGE CONVERSION
        USED TO CONVERT INCOMMING IMAGE MSGS TO CV2 IMAGE AND TAKE
        THRESHHOLD TO DETECT ALL PACKAGES.'''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            rospy.logerr(e)


        th3 = cv2.adaptiveThreshold(cv_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY, 11, 4) #15,6



        shelf_info_relative_position = self.get_qr_data(th3)
        return shelf_info_relative_position


    def attach_box(self):
        '''ATTACH BOX
        FUNCTION USED TO ATTACH BOX TO UR5 ARM IN  GAZEBO AND RVIZ BOTH.'''
        self.box_pose = geometry_msgs.msg.PoseStamped()
        self.box_pose.header.frame_id = "ur5_wrist_3_link"
        self.box_pose.pose.orientation.w = 1.0
        self.box_pose.pose.position.x = 0.0
        self.box_pose.pose.position.y = 0.2
        self.box_pose.pose.position.z = 0
        self.box_name = "box"
        self._scene.add_box(self.box_name, self.box_pose, size=(0.16, 0.16, 0.16))
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


    def callback_orders(self, data):
        '''CALLBACK ORDERS
        FUNCTION USED TO RECIEVE INCOMMING ORDERS AND PUBLISH INCOMMING ORDERS IN SPREAD
        SHEET USING ROS_IOT_BRIDGE.'''
        incoming_order = json.loads(data.message)

        if incoming_order['item'] == "Clothes":
            spread = ["IncomingOrders", incoming_order['order_id'], incoming_order['order_time'],
                      incoming_order['item'], "LP", incoming_order['qty'],
                      incoming_order['city'], incoming_order['lon'], incoming_order['lat'], "150"]

            goal_handle = self.send_goal_iot('mqtt', "pub", self._config_mqtt_pub_topic, spread)

        if incoming_order['item'] == "Medicine":
            spread = ["IncomingOrders", incoming_order['order_id'], incoming_order['order_time'],
                      incoming_order['item'], "HP", incoming_order['qty'], incoming_order['city'],
                      incoming_order['lon'], incoming_order['lat'], "450"]

            goal_handle = self.send_goal_iot('mqtt', "pub", self._config_mqtt_pub_topic, spread)

        if incoming_order['item'] == "Food":
            spread = ["IncomingOrders", incoming_order['order_id'], incoming_order['order_time'],
                      incoming_order['item'], "MP", incoming_order['qty'], incoming_order['city'],
                      incoming_order['lon'], incoming_order['lat'], "250"]

            goal_handle = self.send_goal_iot('mqtt', "pub", self._config_mqtt_pub_topic, spread)
        self.orders_list.append(json.loads(data.message))
        print self.orders_list


    def proceed(self):
        '''PROCEED FUNCTION
        FUNCTION USED FOR PICK PLACE ACTION FOR DIAPATCHING ORDERS.'''
        dicto = self.orders_list.pop(0)

        if dicto['item'] == "Clothes":

            spread = ["OrdersDispatched", dicto['order_id'], dicto['city'], dicto['item'], "LP",
                      dicto['qty'], "150", "Yes", self.get_time_str()]

            rospy.loginfo("picking packagen_"+str(self.green_pkgs[0][1])+str(self.green_pkgs[0][2]))
            self.hard_set_joint_angles(eval("self."+"packagen_"+
            	                               str(self.green_pkgs[0][1])+
            	                               str(self.green_pkgs[0][2])), 5)
            self.attach_box()
            self.moveit_hard_play_planned_path_from_file(self._file_path,
            	                                            "pkg"+str(self.green_pkgs[0][1])+
            	                                            str(self.green_pkgs[0][2])+
            	                                            "_to_drop.yaml",
            	                                            5)
            self.detach_box()

            goal_handle = self.send_goal_iot('mqtt', "pub", self._config_mqtt_pub_topic, spread)
            rospy.sleep(2)
            data = [self.green_pkgs[0][0], json.dumps(spread)]
            self.color.publish(data)
            del self.green_pkgs[0]


        if dicto['item'] == "Medicine":

            spread = ["OrdersDispatched", dicto['order_id'], dicto['city'], dicto['item'], "HP",
                      dicto['qty'], "450", "Yes", self.get_time_str()]

            rospy.loginfo("picking packagen_" +str(self.red_pkgs[0][1])+str(self.red_pkgs[0][2]))
            self.hard_set_joint_angles(eval("self."+"packagen_"+
            	                               str(self.red_pkgs[0][1])+str(self.red_pkgs[0][2])),
                                       5)
            self.attach_box()
            self.moveit_hard_play_planned_path_from_file(self._file_path,
            	                                            "pkg"+str(self.red_pkgs[0][1])+
            	                                            str(self.red_pkgs[0][2])+
            	                                            "_to_drop.yaml",
            	                                            5)
            self.detach_box()

            goal_handle = self.send_goal_iot('mqtt', "pub", self._config_mqtt_pub_topic, spread)
            rospy.sleep(2)
            data = [self.red_pkgs[0][0], json.dumps(spread)]
            self.color.publish(data)
            del self.red_pkgs[0]


        if dicto['item'] == "Food":

            spread = ["OrdersDispatched", dicto['order_id'], dicto['city'], dicto['item'], "MP",
                      dicto['qty'], "250", "Yes", self.get_time_str()]

            rospy.loginfo("picking packagen_"+
            	             str(self.yellow_pkgs[0][1])+str(self.yellow_pkgs[0][2]))
            self.hard_set_joint_angles(eval("self."+"packagen_"+
            	                               str(self.yellow_pkgs[0][1])+
            	                               str(self.yellow_pkgs[0][2])), 5)
            self.attach_box()
            self.moveit_hard_play_planned_path_from_file(self._file_path,
            	                                            "pkg"+str(self.yellow_pkgs[0][1])+
            	                                            str(self.yellow_pkgs[0][2])+
            	                                            "_to_drop.yaml",
            	                                            5)
            self.detach_box()

            goal_handle = self.send_goal_iot('mqtt', "pub", self._config_mqtt_pub_topic, spread)
            rospy.sleep(2)
            data = [self.yellow_pkgs[0][0], json.dumps(spread)]
            self.color.publish(data)
            del self.yellow_pkgs[0]


    def get_time_str(self):
        '''GET TIME STR FUNCTION
        FUNCTION IS USED TO GET DISPATCH TIME IN STRING FORMAT.'''
        timestamp = int(time.time())
        value = datetime.datetime.fromtimestamp(timestamp)
        str_time = value.strftime('%Y-%m-%d %H:%M:%S')

        return str_time


    # Destructor

    def __del__(self):
        '''DESTRUCTOR
        TO DELETE CLASS OBJECT .'''
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():
    '''MAIN FUNCTION
        IT FIRST CAPTURES IMAGE FROM 2D CAMERA AND GETS ALL PACKAGES ON  SHELF AND  PUSH ALL
        PACKAGES ON IMS SHEET AND USED TO PROCSS INCOMMING ORDES.'''

    ur5 = Ur5Moveit("ur5_1")
    rospy.sleep(5)

    msg = rospy.wait_for_message("/eyrc/vb/camera_1/image_raw", Image)
    qr_data_list = ur5.image_conversion(msg)
    print qr_data_list
    date = datetime.datetime.today()
    for x in range(0, len(qr_data_list)):

        if qr_data_list[x][0] == "yellow":
            pkg = ["Inventory",
                   "Y"+str(qr_data_list[x][1])+str(qr_data_list[x][2])+
                   date.strftime("%m")+date.strftime("%y"),
                   "Food", "MP", "R"+str(qr_data_list[x][1])+" "+"C"+str(qr_data_list[x][2]),
                   "250", "1"]
            goal_handle = ur5.send_goal_iot('mqtt', "pub", ur5._config_mqtt_pub_topic, pkg)
            ur5._goal_handles[x] = goal_handle
            ur5.yellow_pkgs.append(qr_data_list[x])
            rospy.sleep(2)


        if qr_data_list[x][0] == "green":
            pkg = ["Inventory", "G"+str(qr_data_list[x][1])+str(qr_data_list[x][2])+
                   date.strftime("%m")+date.strftime("%y"),
                   "Clothes", "LP", "R"+str(qr_data_list[x][1])+" "+"C"+str(qr_data_list[x][2]),
                   "150", "1"]
            goal_handle = ur5.send_goal_iot('mqtt', "pub", ur5._config_mqtt_pub_topic, pkg)
            ur5._goal_handles[x] = goal_handle
            ur5.green_pkgs.append(qr_data_list[x])
            rospy.sleep(2)


        if qr_data_list[x][0] == "red":
            pkg = ["Inventory", "R"+str(qr_data_list[x][1])+str(qr_data_list[x][2])+
                   date.strftime("%m")+date.strftime("%y"),
                   "Medicine", "HP", "R"+str(qr_data_list[x][1])+" "+"C"+str(qr_data_list[x][2]),
                   "450", "1"]
            goal_handle = ur5.send_goal_iot('mqtt', "pub", ur5._config_mqtt_pub_topic, pkg)
            ur5._goal_handles[x] = goal_handle
            ur5.red_pkgs.append(qr_data_list[x])
            rospy.sleep(2)

    while True:
        if len(ur5.orders_list) > 0:
            if len(ur5.orders_list) >= 2:
                ur5.orders_list.sort(key=lambda i: i['order_id'])
                ur5.proceed()

            else:
                ur5.proceed()

    rospy.spin()

    del ur5



if __name__ == '__main__':

    main()
