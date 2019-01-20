import rospy
import math
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from goal_publisher.msg import PointArray

class T1000:
    def __init__(self):
        self.goal = [] #all goals are stored here
        rospy.init_node('skynet') #the node is initialized
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #publishes the velocity commands
        self.pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.get_state) # reads xyz of a Robot from ModelStates
        self.goals_sub = rospy.Subscriber('/goals', PointArray, self.get_goals) # reads xyz of the goals from PointArray
        self.rate = rospy.Rate(10) #rate for the loop 10Hz
    def get_state(self,data):
        
    def get_goals(self,data):



#make and run the main function
if __name__ == '__main__':
    try:
        var = T1000()
        var.reach_goals()
    except rospy.ROSInterruptException:
        rospy.sleep()
