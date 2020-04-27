#!/usr/bin/env python

import rospy

from std_msgs.msg import Int32
from std_msgs.msg import String

var=None

#define the display text
def callback(msg):
	global var
	var=msg.data


if __name__=='__main__':
	rospy.init_node('random_LED')
	rospy.Subscriber('random_number',Int32, callback)
	pub=rospy.Publisher('LED', String, queue_size=1)
	rate=rospy.Rate(10)

	while not rospy.is_shutdown():
		if var<=2500:
			#send message to turn OFF the LED
			varP="OFF"
			rospy.loginfo("The output is OFF and the var is: %s", \
				var)
		else:
			#send message to turn ON the LED
			varP="ON"
			rospy.loginfo("The output is ON and the var is: %s", \
				var)

		pub.publish(varP)    
		rate.sleep()
