#! /usr/bin/env python

#import os
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from pkg_vb_sim.srv import vacuumGripper  #INCLUDING SERVICE FILES
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse

class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_t2_ur5_1_pick_place', anonymous=True)

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


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')


    '''def set_joint_angles(self, arg_list_joint_angles):
            
                    list_joint_values = self._group.get_current_joint_values()
                    rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
                    rospy.loginfo(list_joint_values)
            
                    self._group.set_joint_value_target(arg_list_joint_angles)
                    self._group.plan()
                    flag_plan = self._group.go(wait=True)
            
                    list_joint_values = self._group.get_current_joint_values()
                    rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
                    rospy.loginfo(list_joint_values)
            
                    pose_values = self._group.get_current_pose().pose
                    rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
                    rospy.loginfo(pose_values)
            
                    if (flag_plan == True):
                        rospy.loginfo(
                            '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
                    else:
                        rospy.logerr(
                            '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')
            
                    return flag_plan'''





    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m') 



    def add_box(self, timeout=4):

        self.box_pose = geometry_msgs.msg.PoseStamped()
        self.box_pose.header.frame_id = "ur5_pedestal_link"
        self.box_pose.pose.orientation.w = 1.0
        self.box_pose.pose.position.x = 0.01
        self.box_pose.pose.position.y = 0.50
        self.box_pose.pose.position.z = 1.910 
        self.box_name = "box"
        self._scene.add_box(self.box_name, self.box_pose, size=(0.2, 0.2, 0.2))



    def attach_box(self, timeout=4):
        touch_links = self._eef_link
        self._scene.attach_box(self._eef_link, self.box_name, touch_links=touch_links)   
        req_attach = vacuumGripperRequest()
        req_attach.activate_vacuum_gripper = True
        resp = self._activate_vacuum_gripper(req_attach)
        print(resp)
        #os.system('rosservice call /eyrc/vb/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: true"')



    def detach_box(self, timeout=4):

        self._scene.remove_attached_object(self._eef_link, name=self.box_name)
        req_detach = vacuumGripperRequest()
        req_detach.activate_vacuum_gripper = False
        resp = self._activate_vacuum_gripper(req_detach)
        print(resp)
        rospy.sleep(2)
        self._scene.remove_world_object(self.box_name)
        #os.system(' rosservice call /eyrc/vb/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: false" ' )

        
    

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()
    ur5.add_box()
    ur5.go_to_predefined_pose("pkg_position")
    ur5.attach_box()
    ur5.go_to_predefined_pose("straightUp")
    ur5.go_to_predefined_pose("drop_pose")
    ur5.detach_box()
    ur5.go_to_predefined_pose("allZeros")

        
    del ur5
    rospy.spin()


if __name__ == '__main__':
    main()
    