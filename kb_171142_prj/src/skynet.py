#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from goal_publisher.msg import PointArray
from math import pow, sqrt, degrees

class T1000:
    cur_position = [3]
    w = 3
    h = 2
    goal = [[0 for x in range(w)] for y in range(h)]  #all goals are stored here
    def __init__(self):
        rospy.init_node('skynet') #the node is initialized
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #publishes the velocity commands
        self.pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.get_state) # reads xyz of a Robot from ModelStates
        self.goals_sub = rospy.Subscriber('/goals', PointArray, self.get_goals) # reads xyz of the goals from PointArray
        self.rate = rospy.Rate(10) #frequency of the loop 10Hz

    def get_state(self, data): # get the coordinates and deflection of a robot and stores them in to the array
        rate = rospy.Rate(4) #frequency of the loop 4Hz
        quaternion = (round((data.pose[1].orientation.x),3), round((data.pose[1].orientation.y),3), round((data.pose[1].orientation.z),3), round((data.pose[1].orientation.w),3))
        self.cur_position = (round((data.pose[1].position.x), 3), round(( data.pose[1].position.y), 3), round((degrees(euler_from_quaternion(quaternion)[2])), 3))

        #print ('position X is: ' +str(self.cur_position[0]))
        #print ('position Y is: ' +str(self.cur_position[1]))
        #print ('angle    Z is: ' +str(self.cur_position[2]) +'\n')
        rate.sleep()

    def get_goals(self, data):
        rate = rospy.Rate(4) #frequency of the loop 4Hz
        (self.goal[0][0], self.goal[1][0]) = (data.goals[0].x, data.goals[0].y)
        (self.goal[0][1], self.goal[1][1]) = (data.goals[1].x, data.goals[1].y)
        (self.goal[0][2], self.goal[1][2]) = (data.goals[2].x, data.goals[2].y)


        #print('goal 1 = ' + str(self.goal[0][0]) + ', ' + str(self.goal[1][0]))
        #print('goal 2 = ' + str(self.goal[0][1]) + ', ' + str(self.goal[1][1]))
        #print('goal 3 = ' + str(self.goal[0][2]) + ', ' + str(self.goal[1][2]))
        rate.sleep()

    def reach_goals(self):
        rospy.spin()

#make and run the main function
if __name__ == '__main__':
    try:
        var1 = T1000()
        var1.reach_goals()
    except rospy.ROSInterruptException:
        pass
