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
#include <std_msgs/Bool.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <visualization_msgs/Marker.h>

using namespace std;

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
Eigen::Matrix4f Tm180;
float picked_x = 0.0;
float picked_y = 0.0;
float picked_z = 0.0;
bool picked (false);
bool flipped (false);
cloudrgbptr cloud_source (new cloudrgb());	//Loaded source cloud - should I put this in the align callback?
cloudrgbptr cloud_translated (new cloudrgb());	//Translated cloud using point
cloudxyzptr cloud_arrow (new cloudxyz());	//points for arrow

void
flipped_cb (const std_msgs::Bool::ConstPtr& msg)
{
   if (msg->data) flipped = true;
   else flipped = false;
}

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
  if (picked)
  {  
	cloudrgbptr cloud_segment (new cloudrgb());	//Segmented point cloud from msg
  	pcl::fromROSMsg(*cloud, *cloud_segment); 	//Convert msg to point cloud
  	
	cloudrgbptr cloud_aligned (new cloudrgb());  	//Aligned output from ICP
  	sensor_msgs::PointCloud2 cloud_aligned_msg;  	//create msg to publish

	// Translate       	
	Eigen::Matrix4f Tm;
	Tm << 	1, 0, 0, picked_x,
			0, 1, 0, picked_y,
			0, 0, 1, picked_z,
			0, 0, 0, 1;
	if (flipped) Tm = Tm * Tm180; //flip it if requested per topic
	pcl::transformPointCloud(*cloud_source, *cloud_translated, Tm);

	// ICP

	// Setup ICP
  	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  	icp.setEuclideanFitnessEpsilon (1e-10);
	icp.setTransformationEpsilon (1e-6);
  	icp.setMaxCorrespondenceDistance (.1);
  	icp.setMaximumIterations(50);
  	icp.setInputSource(cloud_translated);
  	icp.setInputTarget(cloud_segment);

  	//copy the source cloud
  	cloud_aligned = cloud_translated;
	Eigen::Matrix4f prev;
 
  	//Forced iteractions with ICP loop
  	for (int i = 0; i < 5; ++i)
  	{
    	PCL_INFO ("Iteration Nr. %d.\n", i);
    	cloud_translated = cloud_aligned; //when you set to pointers equal does it actaully set the point clouds equal?
    	icp.setInputSource (cloud_translated);
    	icp.align (*cloud_aligned);
	  	Tm = icp.getFinalTransformation () * Tm;
    	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	  	//added change to each iteration
	  	if (fabs ((icp.getLastIncrementalTransformation () - prev).sum ()) < icp.getTransformationEpsilon ())
      		icp.setMaxCorrespondenceDistance (icp.getMaxCorrespondenceDistance () - 0.001);
    	prev = icp.getLastIncrementalTransformation ();
  	}
  	std::cout << "Final Tm:" << std::endl;
  	std::cout << Tm << std::endl;	

  	//Convert the pcl cloud back to rosmsg
  	pcl::toROSMsg(*cloud_aligned, cloud_aligned_msg);
  
  	//Set the header of the cloud
  	cloud_aligned_msg.header.frame_id = cloud->header.frame_id;
  	// Publish the data
  	//You may have to set the header frame id of the cloud_filtered also
  	pub1.publish (cloud_aligned_msg);

	// Create the transform matrix between the origin and the center of the eye (when imported)
  	// NOTE:  when imported the origin is at the top of the lifting eye
  	// This is to help when the lifting eye is translated to the picked point (which will probably
  	// be at the top of the lifting eye.
  	Eigen::Matrix4f centerOffset;
  	centerOffset << 1, 0, 0, 0,
		 			0, 1, 0, 0,
					0, 0, 1, 0.027,
					0, 0, 0, 1;
	
	// copy the original transform in case it is needed to transform the cloud later.
  	Eigen::Matrix4f originalTm = Tm;
  	// calculate center of lifting eye tranform
  	Tm = centerOffset * Tm;

	tf::Vector3 origin;
  	origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));
  	tf::Matrix3x3 tf3d;
  	tf3d.setValue(	static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)), 
					static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)), static_cast<double>(Tm(1,2)), 
					static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)), static_cast<double>(Tm(2,2)));

  	tf::Quaternion TmQt;
  	tf3d.getRotation(TmQt);

  	tf::Transform transform;
  	transform.setOrigin(origin);
  	transform.setRotation(TmQt);
	static tf::TransformBroadcaster br;
  	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_rgb_optical_frame", "lifting_eye"));

  	
	// Transform center of lifing eye line (cloud of 2 points)
	cloudxyzptr cloud_arrow_transformed (new cloudxyz());	//points for arrow
	pcl::transformPointCloud (*cloud_arrow, *cloud_arrow_transformed, Tm);

	// Arrow Marker
	visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp =  ros::Time::now();
    marker.ns = "arrow";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    // Arrow Scale
    marker.scale.x = 0.015;
    marker.scale.y = 0.025;
    //marker.scale.y = 0.1;
    // arrow is red
    marker.color.r = 1.0;
    marker.color.a = 0.5;
	//set center point
    geometry_msgs::Point p0;
    p0.x = cloud_arrow_transformed->points[0].x;
    p0.y = cloud_arrow_transformed->points[0].y;
    p0.z = cloud_arrow_transformed->points[0].z;
    marker.points.push_back(p0);
	//set end point
    geometry_msgs::Point p1;
    p1.x = cloud_arrow_transformed->points[1].x;
    p1.y = cloud_arrow_transformed->points[1].y;
    p1.z = cloud_arrow_transformed->points[1].z;
    marker.points.push_back(p1);


    marker_pub.publish(marker);
	
  }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "picked_icp");
  ros::NodeHandle nh;

  // prep for arrow marker
  pcl::PointXYZ centerPt;
  pcl::PointXYZ arrowEndPt;
  centerPt.x = 0.00;
  centerPt.y = 0.00;
  centerPt.z = 0.00;
  cloud_arrow->push_back(centerPt);
  arrowEndPt.x = 0.00;
  arrowEndPt.y = 0.10;
  arrowEndPt.z = 0.00;
  cloud_arrow->push_back(arrowEndPt);

  //create rotation matrix
  Tm180 << 	-1, 0, 0, 0,
		0, -1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

  // Create a ROS subscriber for the clicked point
  ros::Subscriber sub1 = nh.subscribe ("/clicked_point", 1, point_cb);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub2 = nh.subscribe ("/camera/depth_registered/seg_points", 1, align_cb);

  // Create a ROS subscriber for the flipped boolean
  ros::Subscriber sub3 = nh.subscribe("flip", 10, flipped_cb);

  // Create a ROS publisher for the output point cloud
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/aligned_cloud", 1);

  // Create publisher for the marker
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  // Load PDC file of source (lifting eye)
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/taylor/src/data_pcd/top/lifting_eye_aligned_m.pcd", *cloud_source);

  // Spin
  ros::spin ();
}
