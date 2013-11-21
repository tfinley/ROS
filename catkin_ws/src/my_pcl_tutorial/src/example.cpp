#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

using namespace std;


ros::Publisher pub;


void
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud)
{
        if (pcl::io::savePCDFile("/home/taylor/src/data_pcd/exampleOut.pcd", *cloud) == 0)
        {
            cout << "Saved "  << endl;
	    exit(0);
        }

 
/*
  pcl::PCLPointCloud2 cloud_filtered;

 // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (cloud_filtered);


  // Publish the data
  pub.publish (cloud_filtered);
*/
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
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();

}