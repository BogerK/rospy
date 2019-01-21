#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from goal_publisher.msg import PointArray
from math import pow, sqrt, degrees, atan2

class T1000:
    cur_position = [] #current coordinates is here
    goals = []
    def __init__(self):
        rospy.init_node('skynet') #the node is initialized
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #publishes the velocity commands
        self.pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.get_state) # reads xyz of a Robot from ModelStates
        self.goals_sub = rospy.Subscriber('/goals', PointArray, self.get_goals) # reads xyz of the goals from PointArray
        self.rate = rospy.Rate(10)

    def get_state(self, data): # get the coordinates and deflection of a robot and stores them in to the array
        #rate = rospy.Rate(10) #frequency of the loop 10Hz
        quaternion = (round((data.pose[1].orientation.x),3), round((data.pose[1].orientation.y),3), round((data.pose[1].orientation.z),3), round((data.pose[1].orientation.w),3))
        self.cur_position = (round((data.pose[1].position.x), 3), round(( data.pose[1].position.y), 3), round((degrees(euler_from_quaternion(quaternion)[2])), 3))

        #print ('position X is: ' +str(self.cur_position[0]))
        #print ('position Y is: ' +str(self.cur_position[1]))
        #print ('angle    Z is: ' +str(self.cur_position[2]) +'\n')
        #rate.sleep()

    def get_goals(self, PointArray):
        #rate = rospy.Rate(1) #frequency of the loop 10Hz
        run_once = 0
        while 1:
            if run_once == 0:
                self.goals = PointArray.goals

                #print('goal 1 = ' + str(self.goals[0].x) + ', ' + str(self.goals[0].y))
                #print('goal 2 = ' + str(self.goals[1].x) + ', ' + str(self.goals[1].y))
                #print('goal 3 = ' + str(self.goals[2].x) + ', ' + str(self.goals[2].y))
                run_once = 1
        #rate.sleep()

    def e_dist(self, pos_x, pos_y):
        res = sqrt(pow((pos_x - self.cur_position[0]), 2) + pow((pos_y - self.cur_position[1]), 2))
        return res

    def reach_goals(self):
        speed = Twist()
        while(True):
            if self.goals:
                pos_x = self.goals[0].x
                pos_y = self.goals[0].y
                while self.e_dist(pos_x, pos_y) > 0.1:
                    speed.linear.x = 0.3
                    speed.angular.z = 0
                    self.vel_pub.publish(speed)
            self.rate.sleep()

#make and run the main function
if __name__ == '__main__':
    try:
        var1 = T1000()
        var1.reach_goals()

    except rospy.ROSInterruptException:
        pass
