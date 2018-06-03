#!/usr/bin/env python


import rospy
from std_msgs.msg import Float32MultiArray


if __name__ == "__main__":
	pub = rospy.Publisher('ultrassonic',Float32MultiArray,queue_size = 1)
	rospy.init_node("ultra_sensors",anonymous=True)
	rate = rospy.Rate(10) #10hz
	raios = Float32MultiArray()
	while True:
		raios.data = [gyro_data['x'],gyro_data['y'],gyro_data['z']]
		pub.publish(raios)
