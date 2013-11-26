/*

Originally created by Taylor Finley, 2013.11.19
License: BSD
Description: Testing to subscribe to  clicked_point that is publised from RVIZ

*/

#include <ros/ros.h>
// PCL specific includes
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h> //added 11.18.13
#include <boost/foreach.hpp> //added 11.18.13
#include <pcl/io/pcd_io.h>  //added 11.18.13
//added for hydro:
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Header.h>


using namespace std;


void 
point_cb (const boost::shared_ptr<const geometry_msgs::PointStamped>& point_ptr)
{
 cout << point_ptr->point << endl;
 cout << "done" << endl;
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "print_point_msg");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/clicked_point", 1, point_cb);

  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("/camera/depth_registered/seg_points", 1);

  // Spin
  ros::spin ();
}
