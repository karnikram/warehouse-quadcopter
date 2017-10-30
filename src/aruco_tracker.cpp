#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;
using namespace aruco;

/*class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};
*/

int main(int argc, char **argv)
{
	try
	{

		CameraParameters CamParam;
		cv::Mat InImage;
		cv::Mat InputImage;
		cv_bridge::CvImage cvImage;
		sensor_msgs::Image rosImage;
		std_msgs::Header header;

		ros::init(argc, argv, "aruco_tracker");

		ros::NodeHandle nh;

		ros::Publisher feedPub = nh.advertise<sensor_msgs::Image>("aruco_tracker/feed", 10);

		VideoCapture vreader(0);

		if (!vreader.isOpened())
		{
			cerr << "Could not open input" << endl;
			return -1;
		}

		CamParam.readFromXMLFile("/home/flytpod/Documents/Test/build/quadcamera.yml");

		MarkerDetector MDetector;
		MDetector.setThresholdParams(7, 7);
		MDetector.setThresholdParamRange(2, 0);

		vector<int> ID;
		ID.reserve(50);

		int count = 0;
		int ct = 0;
		int zeros = 0;
		int flag = 0;

		vector<int> prev; //marker ID of previous frame
		prev.reserve(10);
		int prevsize = 1;
		ofstream myFile;
		myFile.open("/home/flytpod/ID.txt");

		do
		{

			vreader.grab();
			vreader >> InputImage;
			Rect r(170, 150, 300, 300);

			Mat InImage = InputImage(r);	 //Cropping center part of image
			CamParam.resize(InImage.size()); //Adjusts parameters to size of image indicated

			vector<Marker> Markers = MDetector.detect(InImage);

			if (Markers.size() == 0)
			{
				zeros = zeros + 1;
				flag = 0;
			}

			else
			{
				if (zeros > 5)
					flag = 1;

				zeros = 0;
			}

			for (unsigned int i = 0; i < Markers.size(); i++)
			{
				count = 0;

				for (unsigned int j = 0; j < prevsize; j++)
				{
					if ((Markers[i].id == prev[j]) && (flag == 0))
						count = count + 1;
				}

				if (count == 0)
				{
					ID.push_back(Markers[i].id);
					myFile << ID[ct] << endl;
					cout << ID[ct] << endl;
					ct = ct + 1;
					flag = 0;
				}
			}

			// for each marker, draw info and its boundaries in the image
			for (unsigned int i = 0; i < Markers.size(); i++)
			{
				prev[i] = Markers[i].id;
				Markers[i].draw(InImage, Scalar(0, 0, 255), 2);
			}

			if (Markers.size() != 0)
				prevsize = static_cast<int>(Markers.size());

			header.stamp = ros::Time::now();

			cvImage = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, InImage);
			cvImage.toImageMsg(rosImage);

			feedPub.publish(rosImage);

			if (waitKey(30) >= 0)
				break;

		} while (ros::ok());

		myFile << endl;
		myFile << endl;
		int c = 0;

		vector<int> MarkerID(ID);
		vector<int> freq;
		freq.reserve(ID.size());

		for (int i = 0; i < MarkerID.size(); i++)
		{
			c = 1;
			for (int j = i + 1; j < MarkerID.size(); j++)
			{
				//		cout << j << endl;
				if (MarkerID[i] == MarkerID[j])
				{
					c++;
					for (int m = j; m < MarkerID.size() - 1; m++)
					{
						MarkerID[m] = MarkerID[m + 1];
					}
					MarkerID.pop_back();
					//		   MarkerID.erase(MarkerID.begin() + j-1);
					j--;
				}
			}
			freq.push_back(c);
		}

		for (int i = 0; i < MarkerID.size(); i++)
		{
			myFile << MarkerID[i] << " : " << freq[i] << endl;
			cout << MarkerID[i] << " : " << freq[i] << endl;
		}

		myFile.close();

		//if (cml["-o"]) cv::imwrite(cml("-o"), InImage);
	}
	catch (std::exception &ex)

	{
		cout << "Exception :" << ex.what() << endl;
	}
}
