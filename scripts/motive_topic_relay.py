#!/usr/bin/env python
#----------------------------------------------------------------------------
# Created By  : Waleed Al-Maawali
# Created Date: 25/04/2022
# version ='1.0'
# ---------------------------------------------------------------------------
""" A ROS node for remapping OptiTrack Motive(TM) messages to PX4 vision pose and limit publishing rate"""
""" This is needed to provide localization for the drone and allow indoor flight"""
# ---------------------------------------------------------------------------

import rospy

# PoseStamped messages are published by "vrpn_client_node"
from geometry_msgs.msg import PoseStamped, Pose, Point

# MAVROS services (not used)
from mavros_msgs.srv import *	

# Header for new messages (not used)
from std_msgs.msg import Header	

# Pose uses Quaternion [x,y,z,w] to describe angular position. 
# The following function is useful to convert to Euler [roll, pitch, yaw] (not used)
from tf.transformations import quaternion_from_euler	

# ---------------------------------------------------------------------------

pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
count=0

def callback(data):
	# The ROS node "vrpn_client_node" is publishing data at a rate of 180Hz (can be modified from OptiTrack Motive)
	# The following code will be sample one data from every 10 data frames, hence the pose data will be sent to the drone at ~14Hz.
	# This is needed because of limited serial communication bandwidth between ROS and the drone.
	global count
	if count > 10:
		pub.publish(data)
		count = 0
	else:
		count = count+1
	

def listerner():
	rospy.init_node('motive_topic_relay', anonymous=False)

	# In the following line, you should change "drone1" to the name of your rigid body that was created in OptiTrack Motive
        rospy.Subscriber("/vrpn_client_node/drone1/pose", PoseStamped, callback)

	rospy.sleep(5)
	rospy.spin()

if __name__ == '__main__':
	try:
		listerner()
	except rospy.ROSInterruptException:
		pass
