#!/usr/bin/python2

import rospy
import rosbag
from visualization_msgs.msg import MarkerArray


bag = rosbag.Bag('../bag/project_data.bag')
i = 0

with rosbag.Bag('../bag/ground_truth.bag','w') as outbag:
	for topic, msg, t in bag.read_messages(topics=['/L01/trajectory_node_list']):
		i = i+1
		if(i == 76441):
			outbag.write(topic,msg,t)
bag.close()

