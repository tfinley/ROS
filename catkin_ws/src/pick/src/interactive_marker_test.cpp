/*

Originally created by Taylor Finley, 2013.11.21
License: BSD
Description: 	This is a test to get a interactive arror working as a button

*/

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

void frameCallback(const ros::TimerEvent&)
{
  static uint32_t counter = 0;
  static bool make = true;

  static tf::TransformBroadcaster br;

  tf::Transform t;

  ros::Time time = ros::Time::now();

  t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter)/140.0) * 2.0));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "moving_frame"));

  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter)/140.0, 0.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "rotating_frame"));
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::cout << "button click" << std::endl;      
}

void makeButtonMarker()
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/camera_rgb_optical_frame";
  //int_marker.pose.position.y = 3.0 ;
  int_marker.scale = 1;

  int_marker.name = "button";
  int_marker.description = "Button\n(Left Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  // Marker marker = makeBox( int_marker );

  	// Arrow Marker
	visualization_msgs::Marker marker;
    	marker.header.frame_id = "/camera_rgb_optical_frame";
    	marker.header.stamp =  ros::Time::now();

    	marker.type = visualization_msgs::Marker::ARROW;
    	marker.action = visualization_msgs::Marker::ADD;
    	marker.id = 0;
    	// Arrow Scale
    	marker.scale.x = 0.25;
    	marker.scale.y = 0.5;
    	marker.scale.z = 0.5;
    	// arrow is red
    	marker.color.r = 1.0;
    	marker.color.a = 0.5;
	//set center point
    	geometry_msgs::Point p0;
    	p0.x = 0.0;
    	p0.y = 3.0;
    	p0.z = 0.0;
    	marker.points.push_back(p0);
	//set end point
    	geometry_msgs::Point p1;
    	p1.x = 0.0;
    	p1.y = 4.0;
    	p1.z = 0.0;
    	marker.points.push_back(p1);

  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);


}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "interactive_marker_test");
  ros::NodeHandle n;

  // create a timer to update the published transforms
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  server.reset( new interactive_markers::InteractiveMarkerServer("interactive_marker_test","",false) );

  ros::Duration(0.1).sleep();

  menu_handler.insert( "First Entry", &processFeedback );
  menu_handler.insert( "Second Entry", &processFeedback );
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Submenu" );
  menu_handler.insert( sub_menu_handle, "First Entry", &processFeedback );
  menu_handler.insert( sub_menu_handle, "Second Entry", &processFeedback );


  makeButtonMarker( );


  server->applyChanges();

  ros::spin();

  server.reset();
}
