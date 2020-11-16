#!/usr/bin/env python
import time
from math import sqrt
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleBot():
    def __init__(self):  ## class defination
        rospy.init_node('turtle_rotate', anonymous=True)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)  # publisher
        self.sub = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)  # suscriber
        self.pose = Pose()
        self.rate = rospy.Rate(100)
        self.cmd_vel = Twist()
        # initial values
        self.t_i = 0
        self.goal_x = 0
        self.goal_y = 0
        self.goal_theta = 0
        self.flag = 0
        self.position_array = [0, 0, 0, 0]
        self.dis = 0

    ##callback function
    def update_pose(self, data):  # callback function
        self.pose = data  # setting pose
        if self.flag == 0:
            self.goal_x = self.pose.x
            self.goal_y = self.pose.y
            self.goal_theta = self.pose.theta
            self.position_array = [0, 0, self.goal_x, self.goal_y]
            self.flag = 1

    #function for moving turtle in 1 complete circle
    def move(self):  # main
        # loop for moving turtle bot in circular path
        self.t_i = time.time()
        while not (abs(self.pose.theta - self.goal_theta) < 0.005 and
                   abs(self.pose.theta - self.goal_theta) != 0):
            self.position_array[0] = self.pose.x
            self.position_array[1] = self.pose.y
            # calculating distance travelled by the turtle
            self.dis = self.dis + sqrt(pow((self.position_array[2] - self.position_array[0]), 2)
                                       + pow((self.position_array[3] - self.position_array[1]), 2))
            # updating values in position array to store current and past pos
            self.position_array[2] = self.position_array[0]
            self.position_array[3] = self.position_array[1]
            self.cmd_vel.linear.x = 1  # turtle bot's linear velocity
            self.cmd_vel.angular.z = 1  # turtle bot's  angular velocity
            # logging output
            rospy.loginfo("Moving in circle\n {}".format(self.dis))
            # publishing speed output
            self.pub.publish(self.cmd_vel)
            self.rate.sleep()

        # loop over
        # stopping turtle bot
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.pub.publish(self.cmd_vel)
        # print (self.pose.x , self.pose.y , self.pose.theta)
        # final message
        rospy.loginfo("goal reached")
        rospy.spin()


if __name__ == '__main__':
    try:
        # creating class object
        C = TurtleBot()
        # calling move() function
        C.move()
    except rospy.ROSInterruptException:
        pass
