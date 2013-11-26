#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZRGB>);

  //
  //  Load clouds
  //
  if (pcl::io::loadPCDFile (argv[1], *cloud_source) < 0)
  {
    std::cout << "Error loading model cloud." << std::endl;
    return (-1);
  }
  if (pcl::io::loadPCDFile (argv[2], *cloud_target) < 0)
  {
    std::cout << "Error loading scene cloud." << std::endl;
    return (-1);
  }

  //setup ICP
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setEuclideanFitnessEpsilon (1e-12);
  icp.setMaxCorrespondenceDistance (0.5);
  icp.setMaximumIterations(1000);
  icp.setInputSource(cloud_source);
  icp.setInputTarget(cloud_target);

  //copy the source cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_aligned = cloud_source;
  cloud_source = cloud_source_aligned;
  icp.align (*cloud_source_aligned);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
 

  std::cout << icp.getFinalTransformation() << std::endl;
  pcl::io::savePCDFileBinary ("output.pcd", *cloud_source_aligned);
  std::cout << "Euclidean Fitness Epsilon: "<< icp.getFitnessScore() << std::endl;
 return (0);
}
