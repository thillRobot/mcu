/*
 * rosserial Publisher Example
 * Prints "hello world!"
 * to test run the following
 * ``` 
 *   roscore
 *   rosrun rosserial_arduino serial_node.py _port:=/dev/ttyUSB0
 *   rostopic echo chatter
 * ```
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
