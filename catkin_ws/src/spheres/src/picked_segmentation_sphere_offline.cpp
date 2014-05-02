/*

Originally created by Taylor Finley, 2013.12.02
License: BSD
Description: 	Segment a point cloud around a two spheres picked from RVIZ


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


using namespace std;

//global variables
ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
float picked1_x = 0.0;
float picked1_y = 0.0;
float picked1_z = 0.0;
float picked2_x = 0.0;
float picked2_y = 0.0;
float picked2_z = 0.0;
bool picked1 (false);
bool picked2 (false);
int i = 1;

//typedefs
typedef pcl::PointXYZRGB rgbpoint;
typedef pcl::PointCloud<pcl::PointXYZRGB> cloudrgb;
typedef cloudrgb::Ptr cloudrgbptr;

void 
point_cb (const boost::shared_ptr<const geometry_msgs::PointStamped>& point_ptr)
{
  if (i == 1){
  picked2 = false;
  picked1_x = point_ptr->point.x;
  picked1_y = point_ptr->point.y;
  picked1_z = point_ptr->point.z;  
  cout << "picked1" << endl;
  picked1 = true;
  i = i++;
  return;
  }
  
  if (i == 2){
  picked2_x = point_ptr->point.x;
  picked2_y = point_ptr->point.y;
  picked2_z = point_ptr->point.z;  
  cout << "picked2" << endl;
  picked2 = true;
  i = 1;
  return;
  }
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  if (picked1 && picked2)
  {  
	cloudrgbptr PC (new cloudrgb());
  	cloudrgbptr PC_filtered_sphere1 (new cloudrgb());
  	cloudrgbptr PC_filtered_sphere2 (new cloudrgb());
  	pcl::fromROSMsg(*cloud, *PC); //Now you can process this PC using the pcl functions 
  	sensor_msgs::PointCloud2 cloud_msg_sphere1;
  	sensor_msgs::PointCloud2 cloud_msg_sphere2;
	geometry_msgs::Point pickedPoint1;
    geometry_msgs::Point pickedPoint2;

  	//calculate bounding box
  	float delta = 0.075;
  	float x1min = picked1_x - delta;
  	float x1max = picked1_x + delta;
  	float y1min = picked1_y - delta;
  	float y1max = picked1_y + delta;
  	float z1min = picked1_z - delta;
  	float z1max = picked1_z + delta;
  	float x2min = picked2_x - delta;
  	float x2max = picked2_x + delta;
  	float y2min = picked2_y - delta;
  	float y2max = picked2_y + delta;
  	float z2min = picked2_z - delta;
  	float z2max = picked2_z + delta;

	// set points for messages
    pickedPoint1.x = picked1_x;
    pickedPoint1.y = picked1_y;
    pickedPoint1.z = picked1_z;
    pickedPoint2.x = picked2_x;
    pickedPoint2.y = picked2_y;
    pickedPoint2.z = picked2_z;
	
  	//----------------------------------------
  	//------Start Passthrough Filters----------
  	//----------------------------------------

	// SPHERE 1

  	// Create the filtering object x
  	pcl::PassThrough<pcl::PointXYZRGB> pass_x1;
  	pass_x1.setInputCloud (PC);
  	pass_x1.setFilterFieldName ("x");
  	pass_x1.setFilterLimits (x1min, x1max);
  	//pass_x.setFilterLimitsNegative (true);
  	pass_x1.filter (*PC_filtered_sphere1);

  	// Create the filtering object y
  	pcl::PassThrough<pcl::PointXYZRGB> pass_y1;
  	pass_y1.setInputCloud (PC_filtered_sphere1);
  	pass_y1.setFilterFieldName ("y");
  	pass_y1.setFilterLimits (y1min, y1max);
  	//pass_y.setFilterLimitsNegative (true);
  	pass_y1.filter (*PC_filtered_sphere1);

  	// Create the filtering object z
  	pcl::PassThrough<pcl::PointXYZRGB> pass_z1;
  	pass_z1.setInputCloud (PC_filtered_sphere1);
  	pass_z1.setFilterFieldName ("z");
  	pass_z1.setFilterLimits (z1min, z1max);
  	//pass_z.setFilterLimitsNegative (true);
  	pass_z1.filter (*PC_filtered_sphere1);

	// SPHERE 2

  	// Create the filtering object x
  	pcl::PassThrough<pcl::PointXYZRGB> pass_x2;
  	pass_x2.setInputCloud (PC);
  	pass_x2.setFilterFieldName ("x");
  	pass_x2.setFilterLimits (x2min, x2max);
  	//pass_x.setFilterLimitsNegative (true);
  	pass_x2.filter (*PC_filtered_sphere2);

  	// Create the filtering object y
  	pcl::PassThrough<pcl::PointXYZRGB> pass_y2;
  	pass_y2.setInputCloud (PC_filtered_sphere2);
  	pass_y2.setFilterFieldName ("y");
  	pass_y2.setFilterLimits (y2min, y2max);
  	//pass_y.setFilterLimitsNegative (true);
  	pass_y2.filter (*PC_filtered_sphere2);

  	// Create the filtering object z
  	pcl::PassThrough<pcl::PointXYZRGB> pass_z2;
  	pass_z2.setInputCloud (PC_filtered_sphere2);
  	pass_z2.setFilterFieldName ("z");
  	pass_z2.setFilterLimits (z2min, z2max);
  	//pass_z.setFilterLimitsNegative (true);
  	pass_z2.filter (*PC_filtered_sphere2);

	// ---- both clouds filtered now

  	//Convert the pcl cloud back to rosmsg
  	pcl::toROSMsg(*PC_filtered_sphere1, cloud_msg_sphere1);
  	pcl::toROSMsg(*PC_filtered_sphere2, cloud_msg_sphere2);
  
  	//Set the header of the cloud
  	cloud_msg_sphere1.header.frame_id = cloud->header.frame_id;
  	cloud_msg_sphere2.header.frame_id = cloud->header.frame_id;

  	// Publish the data
  	//You may have to set the header frame id of the cloud_filtered also
  	pub1.publish (cloud_msg_sphere1);
  	pub2.publish (cloud_msg_sphere2);
  	pub3.publish (pickedPoint1);
  	pub4.publish (pickedPoint2);

  }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "picked_segmentation");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the clicked point
  ros::Subscriber sub1 = nh.subscribe ("/clicked_point", 1, point_cb);

  // Create a ROS subscriber for the input point cloud

  ros::Subscriber sub2 = nh.subscribe ("/cloud_pcd", 1, cloud_cb);  //offline from pcd to pointcloud


  // Create a ROS publisher for the output point cloud of sphere 1
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/sphere1/seg_points", 1);

  // Create a ROS publisher for the output point cloud of sphere 2
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/sphere2/seg_points", 1);

  // Create a ROS publisher for the output point cloud of sphere 1
  pub3 = nh.advertise<geometry_msgs::Point> ("/sphere1/picked_point", 1);

  // Create a ROS publisher for the output point cloud of sphere 2
  pub4 = nh.advertise<geometry_msgs::Point> ("/sphere2/picked_point", 1);

  // Spin
  ros::spin ();
}
