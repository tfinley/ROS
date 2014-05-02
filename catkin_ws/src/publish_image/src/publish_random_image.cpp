#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sstream> 

/*
AUTHOR: Taylor Finley
LICENSE: BSD
DESCRIPTION: This node randomly selects a numbered file from folder in package to publish 
*/

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_random_image");
  ros::NodeHandle nh;
	srand(time(NULL)); // help to really randomize integers

	int Number = rand() % 5 + 1; // get random integer between 1 and 5
	string packPath = ros::package::getPath("publish_image"); // find package path path
	string folderPath = "/src/random_images/image_"; // folder of random images is within package folder
	string imageNumber = static_cast<ostringstream*>( &(ostringstream() << Number) )->str(); // convert random integer to string to add to file path
	string extension = ".jpg"; // add extension
	string filePath = packPath + folderPath + imageNumber + extension; // concatinate file path
	cout << folderPath << imageNumber << endl; // print out file 

	cv_bridge::CvImage cv_image; // create cv image object
	cv_image.image = cv::imread(filePath,CV_LOAD_IMAGE_COLOR); // load file into object
	cv_image.encoding = "bgr8"; //manually set encoding. this is where I was having issues without this line.
	sensor_msgs::Image ros_image; // create ROS message object
	cv_image.toImageMsg(ros_image); // convert/save cv image to the ROS message

  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/static_image", 1); //create ROS publisher
  ros::Rate loop_rate(5); // rate

  while (nh.ok()) 
  {
    pub.publish(ros_image);
    loop_rate.sleep();
  }
}




