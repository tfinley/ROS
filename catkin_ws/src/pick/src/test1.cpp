/*

Originally created by Taylor Finley, 2013.11.21
License: BSD
Description: 	This will manually create a two points to create an arrow marker
 It will then publish out the marker to rviz to display the arrow

ARROW DETAILS

Start/End Points

You can also specify a start/end point for the arrow, using the points member. If you put points into the points member, it will assume you want to do things this way.
The point at index 0 is assumed to be the start point, and the point at index 1 is assumed to be the end.
scale.x is the shaft diameter, and scale.y is the head diameter. If scale.z is not zero, it specifies the head length.

*/

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
   




using namespace std;



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "test1");
  ros::NodeHandle nh;
  //ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  //std::string pathTest = "tesing 1 2 3";
  std::string packPath = ros::package::getPath("pick");
	std::string pcdPath = "/src/pcd_files/lifting_eye_aligned_m.pcd";
	pcdPath = packPath + pcdPath;

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {

    ROS_INFO_STREAM(pcdPath);


    r.sleep();

  }
}