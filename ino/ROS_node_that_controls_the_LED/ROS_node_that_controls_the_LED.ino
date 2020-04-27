#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
 
ros::NodeHandle nh;

//this function controls the LED
void messageCb(const std_msgs::String &msg)
{
//  nh.logwarn(msg.data);

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
