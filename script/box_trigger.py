#!/usr/bin/env python

################################################################################
## {Description}: Triggering the Relay
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

from __future__ import print_function
from __future__ import division

import sys
import rospy

from std_msgs.msg import String
from std_msgs.msg import Int32
from self_collect_machine.msg import boxStatus

import csv
import datetime
import cv2

import smbus

i2c = smbus.SMBus(1)
I2C_ADD = 0x09 # Arduino I2C address

class BoxIDTrigger_node:
	def __init__(self):

		self.boxState = boxStatus()
		self.prevI2CData = 0

		# Initializing your ROS Node
		rospy.init_node('box_trigger', anonymous=False)

		# Subscribe Int32 msg
		self.boxID_activation_sub = rospy.Subscriber("/boxID_activation", Int32, self.cbBoxTrigger)

	def cbBoxTrigger(self, msg):

		try:		
			i2c.write_byte(I2C_ADD, msg.data)
			#rospy.sleep(0.5)
		finally:
			i2c.write_byte(I2C_ADD, 0)
			#rospy.sleep(0.5)

def main(args):

	vn = BoxIDTrigger_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("[INFO] BoxIDTrigger_node [OFFLINE]...")

if __name__ == '__main__':
	main(sys.argv)
