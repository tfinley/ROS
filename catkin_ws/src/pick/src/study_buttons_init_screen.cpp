/*

Originally created by Taylor Finley, 2013.11.19
License: BSD
Description: 	Recieve segmented point cloud and picked point
		Translate source cloud to picked point
		Aligned translated source cloud to seg cloud using ICP
		Output Aligned Cloud

*/

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h> 
#include <boost/foreach.hpp> 
#include <pcl/io/pcd_io.h>  
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include "pick/Confirm_Button.h"
#include "pick/Reset_Button.h"
#include <cstdlib>

using namespace std;
using namespace visualization_msgs;


//typedefs
typedef pcl::PointXYZRGB rgbpoint;
typedef pcl::PointCloud<pcl::PointXYZRGB> cloudrgb;
typedef cloudrgb::Ptr cloudrgbptr;
typedef pcl::PointCloud<pcl::PointXYZ> cloudxyz;
typedef cloudxyz::Ptr cloudxyzptr;

//global variables
ros::Publisher pub1;
ros::Publisher vis_pub;
ros::Publisher marker_pub;
float picked_x = 0.0;
float picked_y = 0.0;
float picked_z = 0.0;
bool unpicked (true);
bool flipped (false);
bool needsInitialAlign (true);
bool startscreen (true);

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

void 
point_cb (const boost::shared_ptr<const geometry_msgs::PointStamped>& point_ptr)
{
  unpicked = false;
}

void confirmFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      if (startscreen)
      {
		// Create interactive marker for confirm button
		InteractiveMarker int_marker_confirm;
		int_marker_confirm.header.frame_id = "/camera_rgb_optical_frame";
		int_marker_confirm.scale = 1;
		int_marker_confirm.name = "confirm_button";
		int_marker_confirm.description = "Confirm Button";
		InteractiveMarkerControl confirm_control;
		confirm_control.interaction_mode = InteractiveMarkerControl::BUTTON;
		confirm_control.name = "confirm_button_control";

		// confirm button sphere marker
		visualization_msgs::Marker confirm_marker;
		confirm_marker.header.frame_id = "/camera_rgb_optical_frame";
		confirm_marker.header.stamp =  ros::Time::now();
		confirm_marker.type = visualization_msgs::Marker::SPHERE;
		confirm_marker.action = visualization_msgs::Marker::ADD;
		confirm_marker.id = 0;
		// Arrow Scale
		confirm_marker.scale.x = 0.1;
		confirm_marker.scale.y = 0.1;
		confirm_marker.scale.z = 0.1;
		// confirm button is green
		confirm_marker.color.g = 1.0;
		confirm_marker.color.a = 1.0;
		//set center point
		confirm_marker.pose.position.x = 0.9;
		confirm_marker.pose.position.y = -0.7;
		confirm_marker.pose.position.z = 1.7;

		confirm_control.markers.push_back( confirm_marker );
		confirm_control.always_visible = true;
		int_marker_confirm.controls.push_back(confirm_control);
		// Publish to server
		server->insert(int_marker_confirm);
		server->setCallback(int_marker_confirm.name, &confirmFeedback);
		server->applyChanges();

      	startscreen = false;

      	ROS_INFO_STREAM( s.str() << ": button click"  << "." );
		ros::NodeHandle nh;
		ros::ServiceClient confirm_client = nh.serviceClient<pick::Confirm_Button>("confirm_button");
		pick::Confirm_Button srv;
		srv.request.button = "pressed";
		if (confirm_client.call(srv))
		{
		  std::cout << "service call success" << std::endl;
		}
		else
		{
		  ROS_ERROR("Failed to call service confirm_button");
		}
      }
      else
      {
      	ROS_INFO_STREAM( s.str() << ": button click"  << "." );
		ros::NodeHandle nh;
		ros::ServiceClient confirm_client = nh.serviceClient<pick::Confirm_Button>("confirm_button");
		pick::Confirm_Button srv;
		srv.request.button = "pressed";
		if (confirm_client.call(srv))
		{
		  std::cout << "service call success" << std::endl;
		}
		else
		{
		  ROS_ERROR("Failed to call service confirm_button");
		}
      }

      break;
  }
  server->applyChanges();
}

void resetFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click"  << "." );
	  ros::NodeHandle nh;
	  ros::ServiceClient reset_client = nh.serviceClient<pick::Reset_Button>("reset_button");
	  pick::Reset_Button srv;
	  srv.request.button = "pressed";
	  if (reset_client.call(srv))
	  {
	    std::cout << "service call success" << std::endl;
	  }
	  else
	  {
	    ROS_ERROR("Failed to call service reset_button");
	  }

      break;
  }
  server->applyChanges();
}

void
makeMarkers ( ) 
{

	// Create interactive marker for confirm button
	InteractiveMarker int_marker_confirm;
	int_marker_confirm.header.frame_id = "/camera_rgb_optical_frame";
	int_marker_confirm.scale = 1;
	int_marker_confirm.name = "confirm_button";
	int_marker_confirm.description = "Confirm Button";
	InteractiveMarkerControl confirm_control;
	confirm_control.interaction_mode = InteractiveMarkerControl::BUTTON;
	confirm_control.name = "confirm_button_control";

	// confirm button sphere marker
	visualization_msgs::Marker confirm_marker;
	confirm_marker.header.frame_id = "/camera_rgb_optical_frame";
	confirm_marker.header.stamp =  ros::Time::now();
	confirm_marker.type = visualization_msgs::Marker::SPHERE;
	confirm_marker.action = visualization_msgs::Marker::ADD;
	confirm_marker.id = 0;
	// Arrow Scale
	confirm_marker.scale.x = 0.1;
	confirm_marker.scale.y = 0.1;
	confirm_marker.scale.z = 0.1;
	// confirm button is green
	confirm_marker.color.g = 1.0;
	confirm_marker.color.a = 1.0;
	//set center point
	confirm_marker.pose.position.x = 0.0;
	confirm_marker.pose.position.y = 0.0;
	confirm_marker.pose.position.z = 0.5;

	confirm_control.markers.push_back( confirm_marker );
	confirm_control.always_visible = true;
	int_marker_confirm.controls.push_back(confirm_control);
	// Publish to server
	server->insert(int_marker_confirm);
	server->setCallback(int_marker_confirm.name, &confirmFeedback);
	server->applyChanges();

	//***************************************************************

	// Create interactive marker for reset button
	InteractiveMarker int_marker_reset;
	int_marker_reset.header.frame_id = "/camera_rgb_optical_frame";
	int_marker_reset.scale = 1;
	int_marker_reset.name = "reset_button";
	int_marker_reset.description = "Reset Button";
	InteractiveMarkerControl reset_control;
	reset_control.interaction_mode = InteractiveMarkerControl::BUTTON;
	reset_control.name = "reset_button_control";

	// confirm button sphere marker
	visualization_msgs::Marker reset_marker;
	reset_marker.header.frame_id = "/camera_rgb_optical_frame";
	reset_marker.header.stamp =  ros::Time::now();
	reset_marker.type = visualization_msgs::Marker::SPHERE;
	reset_marker.action = visualization_msgs::Marker::ADD;
	reset_marker.id = 1;
	// Arrow Scale
	reset_marker.scale.x = 0.1;
	reset_marker.scale.y = 0.1;
	reset_marker.scale.z = 0.1;
	// sphere is red
	reset_marker.color.r = 1.0;
	reset_marker.color.a = 1.0;
	//set center point
	reset_marker.pose.position.x = -0.9;
	reset_marker.pose.position.y = -0.7;
	reset_marker.pose.position.z = 1.7;

	reset_control.markers.push_back( reset_marker );
	reset_control.always_visible = true;
	int_marker_reset.controls.push_back(reset_control);
	// Publish to server
	server->insert(int_marker_reset);
	server->setCallback(int_marker_reset.name, &resetFeedback);
	server->applyChanges();
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "study_buttons");
  ros::NodeHandle nh;

  // Subscribe to clicked point publisher from RVIZ
  ros::Subscriber sub1 = nh.subscribe ("/clicked_point", 1, point_cb);

  server.reset( new interactive_markers::InteractiveMarkerServer("study_buttons","",false) );
  ros::Duration(0.1).sleep();

  while (unpicked) {
  	ros::spinOnce();
  }

  makeMarkers();

  ros::spin ();
  server.reset();
}
