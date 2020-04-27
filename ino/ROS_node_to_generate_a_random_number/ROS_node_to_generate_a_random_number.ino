#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>

int min=1;
int max=5000;
int rand_no;

ros::NodeHandle nh;
std_msgs::Int32 rand_msg;
ros::Publisher pub_random("/random_number", &rand_msg);
 
char frameid[] = "/randomData";
 
//this function returns the random number
int random_number()
{
  rand_no= random(min, max);
  return rand_no;
}

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
