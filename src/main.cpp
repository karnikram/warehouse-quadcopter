#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

std::vector<cv::Point2f> findFeatures(cv::Mat &src, cv::Mat &srcGray)
{
	std::vector<cv::Point2f> corners;
	int maxCorners = 500;
	double qualityLevel = 0.01;
	double minDistance = 10;
	int blockSize = 3;
	bool useHarrisDetector = false;
	double k = 0.04;

	cv::TermCriteria termCrit(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER,
														20,
														0.03);

	cv::goodFeaturesToTrack(srcGray,
													corners,
													maxCorners,
													qualityLevel,
													minDistance,
													cv::Mat(),
													blockSize,
													useHarrisDetector,
													k);

	cv::cornerSubPix(srcGray,
									 corners,
									 cv::Size(10,10),
									 cv::Size(-1,-1),
									 termCrit);

	//////////////////////Visualisation/////////////////////////

	/*

	std::cout << "Number of corners detected : " << corners.size() << std::endl;

	for(int i = 0; i < corners.size(); i++)
	{
		cv::circle(src, corners[i], 4, cv::Scalar(255,0,0), -1, 8, 0);
	}

	cv::namedWindow("Corners", CV_WINDOW_AUTOSIZE);
	cv::imshow("Corners", src);
	cv::waitKey(0);

	*/

	//////////////////////Visualisation///////////////////////////

	return corners;
}

int main(int argc, char ** argv)
{
	cv::VideoCapture cap(0);
	cv::Mat src, srcGray, prevSrcGray;
	std::vector<cv::Point2f> points[2];

	cv::TermCriteria termCrit(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER,
														20,
														0.03);

	bool init = true;

	cv::namedWindow("Tracking");

  if(!cap.isOpened())
  {
    std::cout << "Camera could not be opened!" << std::endl;
    return -1;
  }

  ros::init(argc, argv, "fyp_node");
  ros::NodeHandle nh;
	int i,k;

	while(1)
	{
		cap.read(src);
		cv::cvtColor(src, srcGray, CV_BGR2GRAY);

		if(init)
		{
			points[0] = findFeatures(src, srcGray);
			init = false;
		}

		if(!points[0].empty())
		{
			std::vector<uchar> status;
			std::vector<float> err;

			if(prevSrcGray.empty())
				srcGray.copyTo(prevSrcGray);

			cv::calcOpticalFlowPyrLK(prevSrcGray,
															srcGray,
															points[0],
															points[1],
															status,
															err,
															cv::Size(31,31),
															3,
															termCrit,
															0,
															0.001);

			for(i = 0, k = 0; i < points[1].size(); i++)
			{
				if(!status[i])
					continue;

				points[1][k++] = points[1][i];

				cv::circle(src, points[1][i], 3, cv::Scalar(255,0,0), -1, 8);
				cv::line(src, points[0][i], points[1][i],	cv::Scalar(0,0,255),	2,	 8);
			}

			points[1].resize(k);
		}

		cv::imshow("Tracking", src);

		if(cv::waitKey(10) == 27)
		{
			 break;
		}

		points[0] = points[1];
		cv::swap(prevSrcGray, srcGray);
	}

		return 0;
}
