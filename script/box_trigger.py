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

class BoxIDTrigger:
	def __init__(self):

		self.boxState = boxStatus()
		self.prevI2CData = 0

		self.trigger_recieved = True

		# Subscribe Int32 msg
		active_topic = "/boxID_activation"
		self.active_sub = rospy.Subscriber(active_topic, Int32, self.cbBoxTrigger)

	def cbBoxTrigger(self, msg):

		try:
			trig = msg.data
		except KeyboardInterrupt as e:
			print(e)

		self.trigger_recieved = True
		self.trigVal = trig

	def push(self):

		if self.trigger_recieved:
			i2c.write_byte(I2C_ADD, self.trigVal)

			# Sleep to give the last log messages time to be sent
			rospy.sleep(1)

			i2c.write_byte(I2C_ADD, 0)

			self.trigger_recieved = False
		else:
			i2c.write_byte(I2C_ADD, 0)

if __name__ == '__main__':

	# Initializing your ROS Node
	rospy.init_node('box_trigger', anonymous=False)
	trigger = BoxIDTrigger()

	# Camera preview
	while not rospy.is_shutdown():
		trigger.push()
