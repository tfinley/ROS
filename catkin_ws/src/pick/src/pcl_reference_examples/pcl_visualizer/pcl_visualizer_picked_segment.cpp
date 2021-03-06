/* \author Geoffrey Biggs */


#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
//TF customization
#include <pcl/visualization/point_picking_event.h>
#include <pcl/console/parse.h>
#include <pcl/filters/crop_box.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/passthrough.h>



//TF Customization added from example to print out xyz from picking of point from cloud

void pp_callback (const pcl::visualization::PointPickingEvent &event, void* viewer_void) 
{ 
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  float picked_x, picked_y, picked_z;      
  if (event.getPointIndex () == -1)
  {
    cout << "pointindex = -1" << endl;
    return;
  }
  if (abs(picked_y) < 0.005)
  {
    cout << "missed point" << endl;
    return;
  }
  event.getPoint(picked_x, picked_y, picked_z);
  cout << picked_x << "  " << picked_y << " " << picked_z << endl;
  //test the control of the viewer
  viewer->setBackgroundColor (1, 1, 1);

  //calculate bounding box
  float delta = 0.1;
  float xmin = picked_x - delta;
  float xmax = picked_x + delta;
  float ymin = picked_y - delta;
  float ymax = picked_y + delta;
  float zmin = picked_z - delta;
  float zmax = picked_z + delta;

  //----------------------------------------
  //------Start Passthrough Filter----------
  //----------------------------------------

  //reopen the point cloud because I dont know how to pass the pointer correctly
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_original_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>& point_cloud_original = *point_cloud_original_ptr;
  pcl::io::loadPCDFile ("/home/taylor/src/data_pcd/top/kinect_top_rgb.pcd", point_cloud_original);  

   std::cout << "Cloud before filtering: " << point_cloud_original_ptr->points.size() << std::endl;

  //create new clouds to save the filtered
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xy (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xyz (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  // Create the filtering object x
  pcl::PassThrough<pcl::PointXYZRGB> pass_x;
  pass_x.setInputCloud (point_cloud_original_ptr);
  pass_x.setFilterFieldName ("x");
  pass_x.setFilterLimits (xmin, xmax);
  //pass_x.setFilterLimitsNegative (true);
  pass_x.filter (*cloud_filtered_x);

  //I'm not sure if I need to create new objects for each passthrough or if I can just reset it each time with new settings.
  //I'm going to be safe and create new objects for each.

  // Create the filtering object y
  pcl::PassThrough<pcl::PointXYZRGB> pass_y;
  pass_y.setInputCloud (cloud_filtered_x);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (ymin, ymax);
  //pass_y.setFilterLimitsNegative (true);
  pass_y.filter (*cloud_filtered_xy);

  // Create the filtering object z
  pcl::PassThrough<pcl::PointXYZRGB> pass_z;
  pass_z.setInputCloud (cloud_filtered_xy);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (zmin, zmax);
  //pass_z.setFilterLimitsNegative (true);
  pass_z.filter (*cloud_filtered_xyz);

  std::cout << "Cloud after filtering: " << cloud_filtered_xyz->points.size() << std::endl;  
  //----------------------------------------
  //--------End Passthrough Filter----------
  //----------------------------------------

  //update viewer with new cloud
  viewer->removeAllPointClouds();
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_filtered_xyz); //what does this do?
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud_filtered_xyz, rgb, "segmentedCloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segmentedCloud");

  //add sphere to previously picked point on top of lifting eye
  pcl::PointXYZ p1;
  p1.x = picked_x;
  p1.y = picked_y;
  p1.z = picked_z;
  viewer->addSphere(p1, 0.005, 1.0, 0.0, 1.0, "PickedPoint", 0);

}

//create shared viewer

boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();
  return (viewer);
}

// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{ 
  // TF---Load PCD file from Kinect 
  // I really dont know how the creation of the pointer and point cloud work here
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr_kinect (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>& point_cloud_kinect = *point_cloud_ptr_kinect;
  pcl::io::loadPCDFile ("/home/taylor/src/data_pcd/top/kinect_top_rgb.pcd", point_cloud_kinect);


  // -----Open 3D viewer and add point cloud-----
  cout << "creating viewer now" << endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = createViewer();

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr_kinect); //what does this do?
  viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr_kinect, rgb, "initial cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "initial cloud");
  viewer->registerPointPickingCallback (pp_callback, (void*)&viewer);

  //TF custom camera position for this point cloud from Kinect to make it look like a 2D picture
  viewer->spinOnce(1000);
  viewer->setCameraPosition(0.00, 0.00, -1.25, 0.00, 0.00, 0.625, -0.00, -0.99999, 0.000);
  viewer->setCameraFieldOfView(0.523599);
  viewer->setCameraClipDistances(0.0, 4.0);
  viewer->setSize(1000,1000);
  viewer->updateCamera(); 
  viewer->spinOnce(100);
  cout << " " << endl;
   


  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (1000));

  }
}
