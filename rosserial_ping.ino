#include <ros.h>
#include <ros/time.h>
#include <NewPing.h>
#include <std_msgs/Float32.h>

#define TRIGGER 2
#define ECHO 3

NewPing sonar(TRIGGER, ECHO, 300);

ros::NodeHandle nh;
std_msgs::Float32 msg;
ros::Publisher pub("ultrasound/distance", &msg);

int inMsec;

void setup()
{
  nh.initNode();
  nh.advertise(pub);
}

void loop()
{
  msg.data = (sonar.convert_cm(sonar.ping_median()) * 0.01);

  pub.publish(&msg);
  nh.spinOnce();

  delay(5);
}
