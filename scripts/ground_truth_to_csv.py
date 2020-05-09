#!/usr/bin/python2

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Pose 
import numpy as np


class convert:
    def __init__(self):
        self.marker_array_sub = rospy.Subscriber('/L01/trajectory_node_list', MarkerArray, self.marker_array_callback)
        self.position = np.zeros([2,1])

    def marker_array_callback(self,marker_msg):
        length = len(marker_msg.markers)
        for i in range(length):
            for j in range(len(marker_msg.markers[i].points)):
                x = marker_msg.markers[i].points[j].x
                y = marker_msg.markers[i].points[j].y
                temp = np.array([[x],[y]])
                self.position = np.append(self.position,temp,axis=1)
        print "Save to csv"
        print self.position
        np.savetxt("/home/kyle/catkin_ws/src/ASEN_Project/bag/ground_truth.csv",self.position,delimiter=",")
        print "Save complete"

def main():
	rospy.init_node('gt', anonymous=True)
	my_ros = convert()
	rospy.spin()

if __name__ == '__main__':	
	try:
		main()
	except rospy.ROSInterruptException:
		pass

