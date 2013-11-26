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
#include <visualization_msgs/Marker.h>




using namespace std;



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "marker_test");
  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {

    visualization_msgs::Marker points, arrow;
    points.header.frame_id = arrow.header.frame_id = "/camera_rgb_optical_frame";
    points.header.stamp = arrow.header.stamp = ros::Time::now();
    points.ns = arrow.ns = "points_and_arrow";
    points.action = arrow.action = visualization_msgs::Marker::DELETE; //how to remove an old one?
    points.action = arrow.action = visualization_msgs::Marker::ADD;

    points.id = 0;
    arrow.id = 1;
 
    points.type = visualization_msgs::Marker::POINTS;
    arrow.type = visualization_msgs::Marker::ARROW;
 
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // Arrow Scale
    arrow.scale.x = 0.015;
    arrow.scale.y = 0.025;
    //arrow.scale.y = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // arrow is red
    arrow.color.r = 1.0;
    arrow.color.a = 0.5;

    geometry_msgs::Point p0;
    p0.x = -0.128;
    p0.y = -0.061;
    p0.z = 1.423;

    //points.points.push_back(p0);
    arrow.points.push_back(p0);

    geometry_msgs::Point p1;
    p1.x = -0.128;
    p1.y = -0.151;
    p1.z = 1.449;

    //points.points.push_back(p1);
    arrow.points.push_back(p1);

    //marker_pub.publish(points);
    marker_pub.publish(arrow);


    r.sleep();

  }
}