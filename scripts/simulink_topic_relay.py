#!/usr/bin/env python
#----------------------------------------------------------------------------
# Created By  : Waleed Al-Maawali
# Created Date: 25/04/2022
# version ='1.0'
# ---------------------------------------------------------------------------
""" A ROS node for remapping software RC joystick values from Simulink to MAVROS """
""" This is needed to provide control commands to the drone"""
""" Note that you may not be able to publish RC values to MAVROS directly from Simulink, as it uses different message type (OverrideRCIn)"""
""" Hence you will need to run this package"""

"""Using the package:
	1) Build a model in Simulink to publish x, y, z, and yaw (RC joystck) values to a topic /rc_matlab. Message type is ColorRGBA.
	2) Run this node
	3) Run Simulink model.
"""

"""Notes:
	1) RC joystick values range from 1000-1900
	2) All channels, except z, are neutral at 1500. The z channel is neutral at 1100.
	3) For safety purposes, set z channel to 1100 when running Simulink model. Otherwise, the drone may fly immediately.
"""
# ---------------------------------------------------------------------------

import rospy

# Simulink will be using ColorRGBA to publish the RC joystick values
from std_msgs.msg import ColorRGBA

# MAVROS is receiving RC joystick values in Override message type
from mavros_msgs.msg import OverrideRCIn

# MAVROS services
from mavros_msgs.srv import *

# Header for new messages (not used)
from std_msgs.msg import Header

# Our node will be publishing to this topic, which is run by MAVROS
pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

# When publishing RC values, we need to keep setting drone to MANUAL mode, as it may change suddenly during flight
rospy.wait_for_service('mavros/set_mode')

def callback(data):
	# Call switching mode service whenevr a message from Simulink arrives
	try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='MANUAL')
	    print "Switching to MANUAL successfully"
        except rospy.ServiceException, e:
            print "Service set_mode call failed: %s. MANUAL Mode could not be set."%e

	# Mapping RC joystick data from Simulink
	x = data.r
	y = data.g
	z = data.b
	yaw = data.a
	rc = OverrideRCIn()
	rc.channels=[0,0,0,0,0,0,0,0]
	for i in range(8):
		rc.channels[i]=1500
	rc.channels[0] = y
	rc.channels[1] = x
	rc.channels[2] = z
	rc.channels[3] = yaw
	pub.publish(rc)

	# For debugging purposes. You may comment this line.
	print "Published: ", rc.channels
	

def listerner():
	rospy.init_node('simulink_topic_relay', anonymous=False)
        rospy.Subscriber("/rc_matlab", ColorRGBA, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		listerner()
	except rospy.ROSInterruptException:
		pass
