// Define for Leonardo
#define USE_USBCON

#include <ros.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;

int sensorPin = 3;

std_msgs::Float64 sipnpuff;
ros::Publisher p("sipnpuff_sensor", &sipnpuff);

void setup()
{
  nh.initNode();
  nh.advertise(p);
}

void loop()
{
  sipnpuff.data = analogRead(sensorPin);
  p.publish(&sipnpuff);
  nh.spinOnce();
  delay(10);
}

