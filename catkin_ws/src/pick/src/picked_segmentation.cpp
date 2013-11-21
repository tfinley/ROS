/*

Originally created by Taylor Finley, 2013.11.18
License: BSD
Description: Segment a point cloud around a specific point picked from RVIZ

*/

#include <ros/ros.h>
// PCL specific includes
// deleted for hydro: #include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//added for hydro:
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& input)
{
  pcl::PCLPointCloud2 cloud_segmented;

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud.makeShared ());
  seg.segment (inliers, coefficients); 
  
  // Publish the model coefficients
  pub.publish (coefficients);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}