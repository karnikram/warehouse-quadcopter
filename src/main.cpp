#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Geometry>
#include <math.h>

#define NO_OF_COLUMNS 320
#define NO_OF_ROWS 240
#define FOV 56*3.14/180

double attRoll, attRollPrev, attPitch, attPitchPrev, attYaw, attYawPrev;
Eigen::Quaternionf eQuaternion;

void getAngles(const geometry_msgs::TwistStamped &eulerData)
{
  attRoll = eulerData.twist.linear.x;
  attPitch = eulerData.twist.linear.y;
  attYaw = eulerData.twist.linear.z;

  eQuaternion = Eigen::AngleAxisf(attRoll,Eigen::Vector3f::UnitX())
      * Eigen::AngleAxisf(attPitch, Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(attYaw, Eigen::Vector3f::UnitZ());

  //ROS_INFO_STREAM_ONCE("Subsribed to Euler Data!");
 // ROS_INFO_STREAM_THROTTLE(1, attRoll<<"\t"<<attPitch<<"\t"<<attYaw);

}

std::vector<cv::Point2f> findFeatures(cv::Mat &src, cv::Mat &srcGray)
{
  std::vector<cv::Point2f> corners;
  int maxCorners = 500;
  double qualityLevel = 0.01;
  double minDistance = 10;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  cv::TermCriteria termCrit(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.03);

  cv::goodFeaturesToTrack(srcGray, corners, maxCorners,
                          qualityLevel, minDistance,	 cv::Mat(),
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
  float compX, compY;
  float ofDispX, ofDispY;
  float altitude = 1;
  float dispX, dispY;
  int medianIdx, medianIdy;

  cv::Point2f currPoint, prevPoint;
  cv::Point2f currPos, prevPos;

  cv::TermCriteria termCrit(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.03);

  bool init = true;
  int init_count = 0;

  int markerId = 0;

  cv::namedWindow("Tracking");

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

  ros::Subscriber sub = nh.subscribe("/flytpod/mavros/imu/data_euler",
               1000, &getAngles);

  ros::Publisher pubPose = nh.advertise<geometry_msgs::PoseStamped>
      ("/flytpod/mavros/vision_pose/pose", 1000);

  ros::Publisher pubMarker = nh.advertise<visualization_msgs::Marker>
      ("visualization_marker", 0);

  geometry_msgs::PoseStamped visionPose;

  visualization_msgs::Marker positionMarker;

  geometry_msgs::Point point;

  visionPose.header.frame_id = "map";

  positionMarker.header.frame_id="map";
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

  int i,k;

  while(1)
  {
    cap.read(src);
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
        cv::line(src, points[0][i], points[1][i],	cv::Scalar(0,0,255), 2, 8);

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

        compX = (attRoll - attRollPrev) * (NO_OF_COLUMNS) / (FOV);
        compY = (attPitch - attPitchPrev) * (NO_OF_ROWS) / (FOV);

        ofDispX = medianX + compX;
        ofDispY = medianY + compY;

        dispX = (2 * altitude * tan(FOV / 2) * ofDispX) / (NO_OF_COLUMNS);
        dispY = (2 * altitude * tan(FOV / 2) * ofDispY) / (NO_OF_ROWS);

        currPos.x = prevPos.x + roundf(dispX * 100) / 100;
        currPos.y = prevPos.y + roundf(dispY * 100) / 100;

        visionPose.pose.position.x = roundf(currPos.x * 100) / 100;
        visionPose.pose.position.y = roundf(currPos.y * 100) / 100;
        visionPose.pose.position.z = altitude;

        visionPose.pose.orientation.x = eQuaternion.x();
        visionPose.pose.orientation.y = eQuaternion.y();
        visionPose.pose.orientation.z = eQuaternion.z();
        visionPose.pose.orientation.w = eQuaternion.w();

        point.x = visionPose.pose.position.x;
        point.y = visionPose.pose.position.y;
        point.z = visionPose.pose.position.z;

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

        pubPose.publish(visionPose);

        //std::cout << dispX << "\t" << dispY << std::endl;
      }

    }

    cv::imshow("Tracking", src);

    if(cv::waitKey(10) == 27)
    {
      break;
    }

    points[0] = points[1];
    cv::swap(prevSrcGray, srcGray);

    attRollPrev = attRoll;
    attPitchPrev = attPitch;
    attYawPrev = attYaw;

    prevPos = currPos;

    ros::spinOnce();
  }

  return 0;
}
