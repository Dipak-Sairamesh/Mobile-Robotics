#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float64

class move_detection(object):    #To find the primary position of the turtlebot
    def __init__(self):
        self._mved_distance = Float64()
        self._mved_distance.data = 0.0
        self.get_initial()

        self.distance_moved_pub = rospy.Publisher('/moved_distance', Float64, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def get_initial(self): #Declaring the initial position
        data_odom = None
        while data_odom is None:
            try:
                data_odom = rospy.wait_for_message("/odom", Odometry, timeout=1)
            except:
                rospy.loginfo("Current odom not ready, retry to set up init pose")

        self._current_position = Point() #Tracking the current position
        self._current_position.x = data_odom.pose.pose.position.x
        self._current_position.y = data_odom.pose.pose.position.y
        self._current_position.z = data_odom.pose.pose.position.z

    def odom_callback(self,msg): #To find the target position
        NewPosition = msg.pose.pose.position
        self._mved_distance.data = self._mved_distance.data + self.compute_dist(NewPosition, self._current_position)
        self.updatecurrent_position(NewPosition)
        if self._mved_distance.data < 0.000001:
            aux = Float64()
            aux.data = 0.0
            self.distance_moved_pub.publish(aux)
        else:
            self.distance_moved_pub.publish(self._mved_distance)

    def updatecurrent_position(self, new_position): #Updating the values of the new position
        self._current_position.x = new_position.x
        self._current_position.y = new_position.y
        self._current_position.z = new_position.z

    def compute_dist(self, new_position, old_position):
        x1 = new_position.x
        x2 = old_position.x
        y1 = new_position.y
        y2 = old_position.y
        dist = math.hypot(x1- x2, y1 – y2)
        return dist
        
    def get_current_pose(self): #To print the current position
    	print("values:",self._current_position)        


    def publish_moved_distance(self):
        rospy.spin() 
if __name__ == '__main__': #To call all the functions
    rospy.init_node('movement_detector_node', anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    speed = Twist()
    speed.linear.x = 0.1275
    movement_obj = move_detection()
    movement_obj.get_current_pose()    
    
    while not rospy.is_shutdown(): #To stop translation as the turtlebot approaches the apriltag
    	pub.publish(speed)
    	if movement_obj._current_position.x > 0.88:
    	    speed.linear.x = 0
    	    pub.publish(speed)
    	


