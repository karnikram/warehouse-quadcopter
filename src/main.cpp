#include <ros/ros.h>
#include <signal.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/ParamSet.h>

#include <Eigen/Geometry>
#include <math.h>
#include <cmath>

#define NO_OF_COLUMNS 320
#define NO_OF_ROWS 240
#define FOV 56*3.14/180

double attRoll, attRollPrev, attPitch, attPitchPrev, attYaw, attYawPrev;
bool sonarInit;
float altitude, prevAltitude;
Eigen::Quaternionf eQuaternion;

void mySigintHandler(int sig)
{

  mavros_msgs::ParamSet::Request req;
  mavros_msgs::ParamSet::Response resp;

  ros::NodeHandle nh;

  ros::ServiceClient ekf2Client = nh.serviceClient<mavros_msgs::ParamSet>
      ("/flytpod/mavros/param/set");
  req.param_id = "EKF2_AID_MASK";
  req.value.integer = 1;
  req.value.real = 1.0;

  while(!ekf2Client.call(req,resp))
  {
    ROS_WARN_STREAM("Still trying to reset EKF2 AID param!");
  }

  req.param_id = "EKF2_HGT_MODE";
  req.value.integer = 0;
  req.value.real = 0.0;

  while(!ekf2Client.call(req,resp))
  {
    ROS_WARN_STREAM("Still trying to reset EKF2 HGT param!");
  }


  ros::shutdown();
}

void getAngles(const geometry_msgs::TwistStamped &eulerData)
{
  attRoll = eulerData.twist.linear.x;
  attPitch = eulerData.twist.linear.y;
  attYaw = eulerData.twist.linear.z;

  eQuaternion = Eigen::AngleAxisf(attRoll,Eigen::Vector3f::UnitX())
      * Eigen::AngleAxisf(attPitch, Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(attYaw, Eigen::Vector3f::UnitZ());
}

void getAltitude(const std_msgs::Float32 &range)
{
  if(!prevAltitude)
    prevAltitude = range.data;

  if(range.data == 0)
    altitude = prevAltitude;

  else
    altitude = range.data;

  sonarInit = true;
  ROS_INFO_STREAM(altitude);
}

std::vector<cv::Point2f> findFeatures(cv::Mat &src, cv::Mat &srcGray)
{
  std::vector<cv::Point2f> corners;
  int maxCorners = 200;
  double qualityLevel = 0.01;
  double minDistance = 20;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  cv::TermCriteria termCrit(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.03);

  cv::goodFeaturesToTrack(srcGray, corners, maxCorners,
                          qualityLevel, minDistance, cv::Mat(),
                          blockSize, useHarrisDetector, k);

  cv::cornerSubPix(srcGray, corners,
                   cv::Size(10,10), cv::Size(-1,-1),
                   termCrit);

  return corners;
}

int main(int argc, char ** argv)
{
  cv::VideoCapture cap(0);
  cv::Mat src, srcGray, prevSrcGray;
  std::vector<cv::Point2f> points[3];
  cv::Mat diffX, diffY, sortX, sortY;

  float medianX, medianY;
  float medianXPrev, medianYPrev;
  float compX, compY;
  float ofDispX, ofDispY;
  float dispX, dispY;
  int medianIdx, medianIdy;

  double dx, dy;

  cv::Point2f currPoint, prevPoint;
  cv::Point2f currPos, prevPos;

  cv::TermCriteria termCrit(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.03);

  bool init = true;
  int init_count = 0;

  int markerId = 0;

 // cv::namedWindow("Tracking");

  if(!cap.isOpened())
  {
    std::cout << "Camera could not be opened!" << std::endl;
    return -1;
  }

  cap.set(CV_CAP_PROP_FPS, 120);
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 240);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 320);

  ros::init(argc, argv, "fyp_node");
  ros::NodeHandle nh;

  ros::Subscriber anglesSub = nh.subscribe("/flytpod/mavros/imu/data_euler",
                                           1000, &getAngles);

  ros::Subscriber rangeSub = nh.subscribe("ultrasound/distance",
                                          1000, &getAltitude);

  ros::Publisher pubPose = nh.advertise<geometry_msgs::PoseStamped>
      ("/flytpod/mavros/vision_pose/pose", 1000);

  geometry_msgs::PoseStamped visionPose;

  geometry_msgs::Point point;

  mavros_msgs::ParamSet::Request req;
  mavros_msgs::ParamSet::Response resp;

  ros::ServiceClient ekf2Client = nh.serviceClient<mavros_msgs::ParamSet>
      ("/flytpod/mavros/param/set");

  req.param_id = "EKF2_AID_MASK";
  req.value.integer = 8;
  req.value.real = 8.0;

  while(!ekf2Client.call(req,resp))
  {
    ROS_WARN_STREAM("Still trying to set EKF2 AID param!");
  }

  req.param_id = "EKF2_HGT_MODE";
  req.value.integer = 0;
  req.value.real = 0.0;

  while(!ekf2Client.call(req,resp))
  {
    ROS_WARN_STREAM("Still trying to set EKF2 HGT param!");
  }

  signal(SIGINT, mySigintHandler);

  visionPose.header.frame_id = "map";

  int i,k;

  cv::Rect rect(110, 70, 100, 100);

  while(ros::ok())
  {
    cap.read(src);

    //cv::transpose(src, src);
    //cv::flip(src, src, 0);

    //src = src(rect);

    cv::cvtColor(src, srcGray, CV_BGR2GRAY);
    init_count++;

    if(init_count > 10 || points[1].size() < 10)
    {
      init = true;
      init_count = 0;
    }

    if(init)
    {
      points[0] = findFeatures(src, srcGray);
      init = false;
    }

    if(!points[0].empty())
    {
      std::vector<uchar> status[2];
      std::vector<float> err[2];

      if(prevSrcGray.empty())
        srcGray.copyTo(prevSrcGray);

      ros::spinOnce();

      dx = attRoll - attRollPrev;
      dy = attPitch - attPitchPrev;

      //if(!(dx >= 0.005 || dx <=-0.005 || dy >=0.005 || dy <=-0.005))
     // {
         cv::calcOpticalFlowPyrLK(prevSrcGray, srcGray,
                               points[0], points[1],
                               status[0], err[0],
                               cv::Size(31,31), 3,
                               termCrit, 0, 0.001);

         cv::calcOpticalFlowPyrLK(prevSrcGray, srcGray,
                               points[1], points[2],
                               status[1], err[1],
                               cv::Size(31,31), 3,
                               termCrit, 0, 0.001);

         for(i = 0, k = 0; i < points[1].size(); i++)
         {
           if(!(status[0][i] || status[1][i]))
             continue;
           points[1][k] = points[1][i];
           points[0][k] = points[0][i];

           k++;
		
           cv::circle(src, points[1][i], 3, cv::Scalar(255,0,0), -1, 8);
           cv::line(src, points[0][i], points[1][i], cv::Scalar(0,0,255), 2, 8);
      }

      points[1].resize(k);
      points[0].resize(k);

      diffX = cv::Mat::zeros(points[1].size(), 1, CV_32S);
      diffY = cv::Mat::zeros(points[1].size(), 1, CV_32S);

      for(i = 0; i < points[1].size(); i++)
      {
        currPoint.x = points[1][i].x;
        currPoint.y = points[1][i].y;

        prevPoint.x = points[0][i].x;
        prevPoint.y = points[0][i].y;

        diffX.at<int>(i) = (int)((currPoint.x - prevPoint.x) * 10000);
        diffY.at<int>(i) = (int)((currPoint.y - prevPoint.y) * 10000);
      }

      if(diffX.rows > 10)
      {
        cv::sortIdx(diffX, sortX, CV_SORT_EVERY_COLUMN || CV_SORT_ASCENDING);
        cv::sortIdx(diffY, sortY, CV_SORT_EVERY_COLUMN || CV_SORT_ASCENDING);

        medianIdx = sortX.at<int>(sortX.rows / 2);
        medianIdy = sortY.at<int>(sortY.rows / 2);

        medianX = (float) diffX.at<int>(medianIdx) / 10000;
        medianY = (float) diffY.at<int>(medianIdy) / 10000;

        compX = dx * (NO_OF_COLUMNS) / (FOV);
        compY = dy * (NO_OF_ROWS) / (FOV);

        ofDispX = medianX - 2.0* compX;
        ofDispY = medianY - compY;

        dispX = (2 * altitude * tan(FOV / 2) * ofDispX) / (NO_OF_COLUMNS);
        dispY = (2 * altitude * tan(FOV / 2) * ofDispY) / (NO_OF_ROWS);

        if(abs(medianX - medianXPrev) < 4.5 && abs(medianY - medianYPrev) < 4.5)
        {
          currPos.x = prevPos.x + dispX;
          currPos.y = prevPos.y + dispY;
        }

        visionPose.pose.position.x = - currPos.x;
        visionPose.pose.position.y =  currPos.y;
        visionPose.pose.position.z = altitude;

        visionPose.pose.orientation.x = eQuaternion.x();
        visionPose.pose.orientation.y = eQuaternion.y();
        visionPose.pose.orientation.z = eQuaternion.z();
        visionPose.pose.orientation.w = eQuaternion.w();

        visionPose.header.stamp = ros::Time::now();

        point.x = visionPose.pose.position.x;
        point.y = visionPose.pose.position.y;
        point.z = visionPose.pose.position.z;
	
        if(sonarInit)
        {
          ROS_INFO_STREAM_ONCE("calculated");
          pubPose.publish(visionPose);
        }

      }

    }
//}

    //cv::imshow("Tracking", src);

  // if(cv::waitKey(5) == 27)
  //  {
 //    break;
  //  }
	
    //cv::waitKey(5);

    medianXPrev = medianX;
    medianYPrev = medianY;
    points[0] = points[1];
    cv::swap(prevSrcGray, srcGray);

    attRollPrev = attRoll;
    attPitchPrev = attPitch;
    attYawPrev = attYaw;

    prevPos = currPos;
    prevAltitude = altitude;

  }

    return 0;
}
