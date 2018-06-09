#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
from geometry_msgs.msg import Twist, Vector3, Pose

def send_vel(Twist):
	linear = Twist.linear.x
	angular = Twist.angular.z
	if(linear > 1):
		linear = 1
	elif(linear < -1):
		linear = -1
	if(angular > 1):
		angular = 1
	elif(angular < -1):
		angular = -1

	angular = angular*255
	linear = linear*255

	#ser.write("L " + str(linear) + "\n")
	#ser.write("A " + str(angular) + "\n")
	print("Angular: ",angular," Linear: ",linear)

rospy.init_node("vel_listener")

while True:


	vel_receiver = rospy.Subscriber("/cmd_vel", Twist, send_vel, queue_size = 1)

	rospy.sleep(0.01)
