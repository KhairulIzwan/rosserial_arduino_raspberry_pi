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
}
```
