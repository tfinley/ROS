// Modified by Taylor Finley
// ./conditional_removal_color_outlier [file name or full path] [red min] [red max] [green min] [ green max] [blue min] [blue max] [outlier distance in mm]
// exmaple: ./conditional_removal_color_outlier /home/taylor/src/data_pcd/spheres/kinect16in0.pcd 125 255 0 120 0 120 8

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl;

int
 main (int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile (argv[1], *cloud) < 0)
  {
    std::cout << "Error loading cloud." << std::endl;
    return (-1);
  }

  //input argv (char) 
  int rMin = atoi(argv[2]);
  int rMax = atoi(argv[3]);
  int gMin = atoi(argv[4]);
  int gMax = atoi(argv[5]);
  int bMin = atoi(argv[6]); 
  int bMax = atoi(argv[7]);

  // build the color condition
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMin)));

  // build the color filter
  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (color_cond);
  condrem.setInputCloud (cloud);
  condrem.setKeepOrganized(true);

  // apply color filter
  condrem.filter (*cloud_filtered);

  //outlier filter
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
  outrem.setInputCloud(cloud_filtered);
  float radius = atoi(argv[8]);
  radius = radius / 1000;
  outrem.setRadiusSearch (radius);
  outrem.setMinNeighborsInRadius (2);
  outrem.filter(*cloud_filtered);
  
  // save cloud after filtering
  if (pcl::io::savePCDFile("filteredCloud.pcd", *cloud_filtered, true) == 0)
  {
     cout << "Saved filteredCloud.pcd " << endl;
  }
  else PCL_ERROR("Problem saving");

  system("pcl_viewer filteredCloud.pcd");

  return (0);
}
