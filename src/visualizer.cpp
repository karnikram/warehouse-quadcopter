#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

geometry_msgs::Point point;

void twistReceived(const geometry_msgs::TwistStamped localPose)
{
  point.x = localPose.twist.linear.x;
  point.y = localPose.twist.linear.y;
  point.z = -localPose.twist.linear.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_visualiser");

  ros::NodeHandle nh;

  ros::Subscriber subPose = nh.subscribe("/flytpod/mavros/local_position/local", 1000, &twistReceived);

  ros::Publisher pubMarker = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate rate(25);

  visualization_msgs::Marker positionMarker;
  int markerId = 0;

  positionMarker.header.frame_id = "map";

  positionMarker.ns = "trajectory";
  positionMarker.header.stamp = ros::Time();

  positionMarker.type = visualization_msgs::Marker::SPHERE_LIST;
  positionMarker.action = visualization_msgs::Marker::ADD;

  positionMarker.scale.x = 0.10;
  positionMarker.scale.y = 0.10;
  positionMarker.scale.z = 0.10;

  positionMarker.color.a = 1.0;
  positionMarker.color.r = 1.0;
  positionMarker.color.g = 0.0;
  positionMarker.color.b = 0.0;

  positionMarker.points.reserve(1000);

  while(ros::ok())
  {
    ros::spinOnce();

    if(positionMarker.points.size() < 1000)
    {
      positionMarker.points.push_back(point);
    }

    else
    {
      positionMarker.points[markerId] = point;
    }

    markerId = ++markerId % 1000;
    pubMarker.publish(positionMarker);

   rate.sleep();
  }

  return 0;
}
