#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from goal_publisher.msg import PointArray

class T1000:
    def __init__(self):
        rospy.init_node('skynet') #the node is initialized
        self.goal = [] #all goals are stored here
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #publishes the velocity commands
        self.pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.get_state) # reads xyz of a Robot from ModelStates
        self.goals_sub = rospy.Subscriber('/goals', PointArray, self.get_goals) # reads xyz of the goals from PointArray
        self.rate = rospy.Rate(10) #rate for the loop 10Hz

    def get_state(self,data):
        self.pose = data.pose[1] #accesses the element #1 of the array of elements in the ModelStates msg (#1 is my Robot)
        self.pose.position.x
        self.pose.position.y

    def get_goals(self,PointArray_msg):
        self.goal = PointArray_msg.goals # accesses the goals from goal_publisher msg

    def e_dist(self): #euclidean distance between pos of a robot and goal point

            return dist

    def reach_goals(self):
        var2 = Twist()
        while True:
            var3.linear.x = 0.3
            var3.angular.z = 0.0
            self.vel_pub.publish(var2)

#make and run the main function
if __name__ == '__main__':
    try:
        var1 = T1000()
        var1.reach_goals()
    except rospy.ROSInterruptException:
        pass
