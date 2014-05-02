/*

Originally created by Taylor Finley, 2013.11.19
License: BSD
Description: 	Recieve segmented point cloud and picked point
		Translate source cloud to picked point
		Aligned translated source cloud to seg cloud using ICP
		Output Aligned Cloud

*/

#include <ros/ros.h>
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
#include <pcl/registration/icp.h>

using namespace std;

//typedefs
typedef pcl::PointXYZRGB rgbpoint;
typedef pcl::PointCloud<pcl::PointXYZRGB> cloudrgb;
typedef cloudrgb::Ptr cloudrgbptr;

//global variables
ros::Publisher pub;
float picked_x = 0.0;
float picked_y = 0.0;
float picked_z = 0.0;
bool picked (false);
cloudrgbptr cloud_source (new cloudrgb());	//Loaded source cloud - should I put this in the align callback?

void 
point_cb (const boost::shared_ptr<const geometry_msgs::PointStamped>& point_ptr)
{
  picked_x = point_ptr->point.x;
  picked_y = point_ptr->point.y;
  picked_z = point_ptr->point.z;  
  cout << "picked" << endl;
  picked = true;
}

void 
align_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "record_study_input");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the clicked point
  ros::Subscriber sub1 = nh.subscribe ("/clicked_point", 1, point_cb);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub2 = nh.subscribe ("/camera/depth_registered/seg_points", 1, align_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/aligned_cloud", 1);

  // Load PDC file of source (lifting eye)
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/taylor/src/data_pcd/top/lifting_eye_aligned_m.pcd", *cloud_source);

  // Spin
  ros::spin ();
}
