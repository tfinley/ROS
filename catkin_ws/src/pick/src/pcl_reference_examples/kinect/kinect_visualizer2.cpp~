#include <iostream> 
#include <boost/thread/thread.hpp> 
#include <pcl/common/common_headers.h> 
#include <pcl/features/normal_3d.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/pcl_visualizer.h> 
#include <pcl/console/parse.h> 
 #include <pcl/io/openni_grabber.h> 

 class SimpleOpenNIViewer 
 { 
   public: 
    
void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) 
     { 
       kinect_cloud=cloud; 
     } 

     void run () 
     { 
       pcl::Grabber* interface = new pcl::OpenNIGrabber(); 

       boost::function<void (const 
pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = 
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1); 
       interface->registerCallback (f); 

       interface->start (); 

          boost::this_thread::sleep (boost::posix_time::seconds (1)); 
       interface->stop (); 
     } 
    
   pcl::PointCloud<pcl::PointXYZ>::ConstPtr get_kinect_cloud(){ 
     return(kinect_cloud); 
   } 

 private: 
   pcl::PointCloud<pcl::PointXYZ>::ConstPtr kinect_cloud; 

 }; 

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) { 
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")); 
  viewer->setBackgroundColor (0, 0, 0); 
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud"); 
  viewer->setPointCloudRenderingProperties 
(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); 
  viewer->addCoordinateSystem (1.0); 
  viewer->initCameraParameters (); 
  return (viewer); 
} 

unsigned int text_id = 0; 
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, 
                            void* viewer_void) { 
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ()) 
  { 
    std::cout << "r was pressed => removing all text" << std::endl; 

    char str[512]; 
    for (unsigned int i = 0; i < text_id; ++i) 
    { 
      sprintf (str, "text#%03d", i); 
      viewer->removeShape (str); 
    } 
    text_id = 0; 
  } 
} 

void mouseEventOccurred (const pcl::visualization::MouseEvent &event, 
                         void* viewer_void) { 
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void); 
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton && 
      event.getType () == 
pcl::visualization::MouseEvent::MouseButtonRelease) 
  { 
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl; 

    char str[512]; 
    sprintf (str, "text#%03d", text_id ++); 
    viewer->addText ("clicked here", event.getX (), event.getY (), str); 
  } 
} 

boost::shared_ptr<pcl::visualization::PCLVisualizer> 
interactionCustomizationVis () 
{ 
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")); 
  viewer->setBackgroundColor (0, 0, 0); 
  viewer->addCoordinateSystem (1.0); 

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer); 
  viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer); 

  return (viewer); 
} 

int 
main () 
{ 

  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>); 

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; 

    viewer = simpleVis(basic_cloud_ptr); 

  SimpleOpenNIViewer v; 
  
  while (!viewer->wasStopped ()) 
  { 
  v.run (); 
    viewer->updatePointCloud(v.get_kinect_cloud(),"sample cloud"); 
    viewer->spinOnce (100); 
    boost::this_thread::sleep (boost::posix_time::microseconds (100000)); 
  } 
} 
