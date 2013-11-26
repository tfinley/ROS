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

int pickedPointIndex;
bool picked (false);

//TF Customization added from example to print out rgb from picking of point from cloud

void pp_callback (const pcl::visualization::PointPickingEvent &event, void* viewer_void) 
{ 
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

  pickedPointIndex = event.getPointIndex();
  if (pickedPointIndex == -1)
  {
    cout << "pointindex = -1" << endl;
    return;
  }
  picked = true;
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
  // TF---Load PCD file 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile (argv[1], *cloud) < 0)
  {
    std::cout << "Error loading cloud." << std::endl;
    return (-1);
  }

  // -----Open 3D viewer and add point cloud-----
  cout << "creating viewer now" << endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = createViewer();

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud); 
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "initial cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "rgb cloud");
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

  //setup picked rgb
  int picked_r, picked_g, picked_b;
  pcl::PointXYZRGB pickedPt;
   
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (1000));

    //print out rgb
    if (picked)
    {
	pickedPt = cloud->points[pickedPointIndex];
	picked_r = pickedPt.r;
	picked_g = pickedPt.g;
	picked_b = pickedPt.b;
        cout << "r: " << picked_r << " g: " << picked_g << " b: " << picked_b << endl;
	pickedPointIndex = -1
	picked = false;
    }
	
  }
}
