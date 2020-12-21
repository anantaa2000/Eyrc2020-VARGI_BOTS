#! /usr/bin/env python

import rospy
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import tf2_ros
import tf2_msgs.msg

import math
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from pkg_vb_sim.srv import vacuumGripper  #INCLUDING SERVICE FILES
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse

from pkg_vb_sim.msg import LogicalCameraImage


class Ur5Moveit():


    # Constructor
    def __init__(self):

        rospy.init_node('node_t3', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        self._activate_vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper) #ROS SERVICE  CLIENT INITIATED
        self._activate_ConveyorBelt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)   #control conveyor
        self._logical_camera_2 = rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, self.callback, queue_size = 1) #to see msg published by  logical cam 
        self.tfBuffer = tf2_ros.Buffer()                            # to initiate tf
        self.listener = tf2_ros.TransformListener(self.tfBuffer)    # to see translation between ant to links 


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m') 

        box_length = 0.15               # Length of the Package
        vacuum_gripper_width = 0.115    # Vacuum Gripper Width
        delta = vacuum_gripper_width + (box_length/2)  # 0.19
        # Teams may use this info in Tasks

        self.ur5_2_home_pose = geometry_msgs.msg.Pose()
        self.ur5_2_home_pose.position.x = -0.8
        self.ur5_2_home_pose.position.y = 0
        self.ur5_2_home_pose.position.z = 1 + vacuum_gripper_width + (box_length/2)
        # This to keep EE parallel to Ground Plane
        self.ur5_2_home_pose.orientation.x = -0.5
        self.ur5_2_home_pose.orientation.y = -0.5
        self.ur5_2_home_pose.orientation.z = 0.5
        self.ur5_2_home_pose.orientation.w = 0.5

        self.red_bin = [math.radians(69),
                          math.radians(-41),
                          math.radians(80),
                          math.radians(-131),
                          math.radians(-91),
                          math.radians(-1)]


        self.green_bin = [math.radians(-3),
                          math.radians(-41),
                          math.radians(80),
                          math.radians(-131),
                          math.radians(-91),
                          math.radians(-1)]

        self.blue_bin = [math.radians(-93),
                          math.radians(-41),
                          math.radians(80),
                          math.radians(-131),
                          math.radians(-91),
                          math.radians(-1)]




    def callback(self, msg):
        # print(msg)
        if (msg.models):
            if (msg.models[0].type == 'ur5'):
                self.conveyor(100)

                if(len(msg.models) >=2):

                    if(msg.models[1].type == 'packagen1'):
                        if (msg.models[1].pose.position.y <= 0.10): 
                           
                            self.conveyor(0)
                            # trans = self.tfBuffer.lookup_transform('logical_camera_2_packagen1_frame', 'ur5_wrist_3_link',  rospy.Time())
                            # self.ee_cartesian_translation(-trans.transform.translation.x, -trans.transform.translation.y, 0 ) 
                            self.attach_box()
                            self.conveyor(11)
                            self.set_joint_angles(self.red_bin)
                            self.detach_box()
                    

                    if(msg.models[1].type == 'packagen2'):
                        if(msg.models[1].pose.position.y <= 0.10):
                            
                            self.conveyor(0)
                            trans = self.tfBuffer.lookup_transform('logical_camera_2_packagen2_frame', 'ur5_wrist_3_link', rospy.Time())
                            self.ee_cartesian_translation(-trans.transform.translation.x, -trans.transform.translation.y, 0)
                            self.attach_box()
                            self.conveyor(11)
                            self.set_joint_angles(self.green_bin)
                            self.detach_box()
                               



                    if(msg.models[1].type == 'packagen3'):
                        if (msg.models[1].pose.position.y <= 0.10):

                            self.conveyor(0)
                            trans = self.tfBuffer.lookup_transform('logical_camera_2_packagen3_frame','ur5_wrist_3_link',rospy.Time())
                            self.ee_cartesian_translation(-trans.transform.translation.x, -trans.transform.translation.y, 0)
                            self.attach_box()
                            self.conveyor(11)
                            self.set_joint_angles(self.blue_bin)
                            self.detach_box()
                             
        

            if (msg.models[0].type == 'packagen1'):
                if (msg.models[0].pose.position.y <= 0.0):
                    self.conveyor(0)
                    self.go_to_pose(self.ur5_2_home_pose)

            if (msg.models[0].type == 'packagen2'):
                if (msg.models[0].pose.position.y <= 0.0):
                    self.conveyor(0)
                    self.go_to_pose(self.ur5_2_home_pose)

            if (msg.models[0].type == 'packagen3'):
                if (msg.models[0].pose.position.y <= 0.0):
                    self.conveyor(0)
                    self.go_to_pose(self.ur5_2_home_pose)


        else :
            self.go_to_pose(self.ur5_2_home_pose)
        


    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5


        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))


        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan) 


    
    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        
        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        
        list_joint_values = self._group.get_current_joint_values()
        
        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan



    def conveyor(self, power):

        req = conveyorBeltPowerMsgRequest()
        req.power = power
        response = self._activate_ConveyorBelt(req)
        print response


    def attach_box(self, timeout=4):
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
        print(resp)


    def detach_box(self, timeout=4):

        self._scene.remove_attached_object(self._eef_link, name=self.box_name)
        req_detach = vacuumGripperRequest()
        req_detach.activate_vacuum_gripper = False
        resp = self._activate_vacuum_gripper(req_detach)
        print(resp)
        self._scene.remove_world_object(self.box_name)
    


    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)
        list_joint_values = self._group.get_current_joint_values()
        pose_values = self._group.get_current_pose().pose
        
        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan



    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def main():

    ur5 = Ur5Moveit() 
    rospy.spin()
    del ur5



if __name__ == '__main__':
    main()
 
