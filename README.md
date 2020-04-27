# rosserial_arduino_raspberry_pi

Arduino is a great development board for reading data from various sensors and 
controlling the robot’s DC motors. Raspberry Pi is good at running ROS on Linux.
So, we can benefit from both systems to build smart robots. The easiest way is 
to connect these boards via USB cable and make them communicate through ROS 
nodes -- (https://www.intorobotics.com/how-to-use-rosserial-with-two-arduinos-and-raspberry-pi/)

# DISCLAIMER
Original code source are from: 
https://www.intorobotics.com/how-to-use-rosserial-with-two-arduinos-and-raspberry-pi/

**Changes**
1. The ROS node to generate a random number:

```
{
#include <ros.h>
#include <std_msgs/Int32.h>

int min=1;
int max=5000;
int rand_no;

ros::NodeHandle nh;
 
//this function returns the random number
int random_number()
{
  rand_no= random(min, max);
  return rand_no;
}

std_msgs::Int32 rand_msg;
ros::Publisher pub_random("/random_number", &rand_msg);

// put your setup code here, to run once:
void setup()
{
  nh.initNode();
  nh.advertise(pub_random);
}

// put your main code here, to run repeatedly:
void loop()
{
  rand_msg.data=random_number();
  pub_random.publish(&rand_msg);
  nh.spinOnce();
  delay(10);
}
```

2. The ROS node that displays and calculates the LED’s stage

```
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
```

3. The ROS node that controls the LED

```
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
 
ros::NodeHandle nh;

//this function controls the LED
void messageCb(const std_msgs::String &msg)
{
  // Convert to a String  
  String state = String(msg.data);
  
  //blink the LED
  if(state == "ON")
  {
    digitalWrite(13, HIGH-digitalRead(13));
  }
  //turn off the LED
  else
  {
    digitalWrite(13, LOW-digitalRead(13));
  }
}

ros::Subscriber<std_msgs::String> sub("LED", messageCb);

// put your setup code here, to run once:
void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

// put your main code here, to run repeatedly:
void loop()
{
  nh.spinOnce();
  delay(10);
}
```

4. Launch file

```
<launch>

	<node	pkg="rosserial_python"
		type="serial_node.py"
		name="twoArduino_LED"
		output="screen">

		<param name="port" value="/dev/ttyUSB0" />
		<param name="baud" value="57600" />
	</node>

	<node	pkg="rosserial_python"
		type="serial_node.py"
		name="twoArduinos_RandNo"
		output="screen">

		<param name="port" value="/dev/ttyUSB1" />
		<param name="baud" value="57600" />
	</node>

	<node	name="random_number"
		pkg="rosserial_arduino_raspberry_pi"
		type="display_calculate_LED_stage.py"
		output="screen">
	</node>
</launch>
```
