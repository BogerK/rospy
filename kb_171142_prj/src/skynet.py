#!/usr/bin/env python

import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from goal_publisher.msg import PointArray
from math import pow, sqrt, degrees, atan2, pi

class T1000:

    def __init__(self):
        rospy.init_node('skynet') #the node is initialized
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #publishes the velocity commands
        self.pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.get_state) # reads xyz of a Robot from ModelStates
        self.goals_sub = rospy.Subscriber('/goals', PointArray, self.get_goals) # reads xyz of the goals from PointArray
        self.rate = rospy.Rate(10)

    def get_state(self, data): # get the coordinates and deflection of a robot and store them in the array
        quaternion = [data.pose[1].orientation.x, data.pose[1].orientation.y, data.pose[1].orientation.z, data.pose[1].orientation.w]
        self.cur_position = [round((data.pose[1].position.x), 3), round(( data.pose[1].position.y), 3), round(euler_from_quaternion(quaternion)[2], 3)]

        #print ('position X is: ' +str(self.cur_position[0]))
        #print ('position Y is: ' +str(self.cur_position[1]))
        #print ('angle    Z is: ' +str(self.cur_position[2]) +'\n')

    def get_goals(self, PointArray): #get the coordinates of the goals and store them in the array
        run_once = 0
        while 1:
            if run_once == 0:
                self.goals = PointArray.goals

                #print('goal 1 = ' + str(self.goals[0].x) + ', ' + str(self.goals[0].y))
                #print('goal 2 = ' + str(self.goals[1].x) + ', ' + str(self.goals[1].y))
                #print('goal 3 = ' + str(self.goals[2].x) + ', ' + str(self.goals[2].y) +'\n')
                run_once = 1

    def e_dist(self, g_pos_x, g_pos_y): #calculates the euclidian distace between the robot and the goal point
        return sqrt(pow((g_pos_x - self.cur_position[0]), 2) + pow((g_pos_y - self.cur_position[1]), 2))

    def robot_angle(self, g_pos_x, g_pos_y): #angle between X-axis, robot,  goal
        return round(atan2(g_pos_y - self.cur_position[1], g_pos_x - self.cur_position[0]), 3)

    def m_line1_angle(self, dest_x, dest_y, orig_x, orig_y): #angle between X-axis, point of origin, goal
        return round(atan2(self.goals[0].y - 0, self.goals[0].x - 0), 3)

    #def deflection(self, )

    def reach_goals(self):
        time.sleep(1)
        speed = Twist()
        while not rospy.is_shutdown():
            #rate=rospy.Rate(10)
            self.e_dist(self.goals[0].x, self.goals[0].y)
            self.robot_angle(self.goals[0].x, self.goals[0].y)
            self.m_line1_angle(self.goals[0].x, self.goals[0].y, 0,0)
            var = self.m_line1_angle(self.goals[0].x, self.goals[0].y, 0,0) - self.robot_angle(self.goals[0].x, self.goals[0].y)
            print('difference is: ' +str(var))
            print('current z is: ' +str(self.cur_position[2]))

            if var <-0.01 and 0.02<=self.cur_position[2]<=3.12:
                speed.linear.x = 0
                speed.angular.z = 0.4
                #self.vel_pub.publish(speed)
            elif var <0.01 and -3.12<=self.cur_position[2]<=-0.02:
                speed.linear.x = 0
                speed.angular.z = -0.4
                #self.vel_pub.publish(speed)
            elif var >0.01 and 0.02<=self.cur_position[2]<=3.12:
                speed.linear.x = 0
                speed.angular.z = -0.4
                #self.vel_pub.publish(speed)
            elif var >0.01 and -3.12<=self.cur_position[2]<=-0.02:
                speed.linear.x = 0
                speed.angular.z = 0.4
                #self.vel_pub.publish(speed)
            else:
                speed.linear.x = 0
                speed.angular.z = 0.0
            self.vel_pub.publish(speed)

            #print('the distance is ' +str(self.e_dist(self.goals[0].x, self.goals[0].y)))
            #print('angle (robot to goal1) is ' +str(self.robot_angle(self.goals[0].x, self.goals[0].y)))
            #print('angle (origin to goal1) is ' +str(self.m_line1_angle(self.goals[0].x, self.goals[0].y, 0,0)))

            #rate.sleep()

        self.rate.sleep()

#make and run the main function
if __name__ == '__main__':
    try:
        var1 = T1000()
        var1.reach_goals()
    except rospy.ROSInterruptException:
        pass
