#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


typedef pcl::PointXYZRGB PointT;

int
main (int argc, char** argv)
{
  // All the objects needed
  pcl::PCDReader reader;
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr cluster_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_sphere (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_sphere (new pcl::PointIndices);
  float x0, x1, y0, y1, z0, z1, r0, r1, d;

  // Read in the cloud data
  reader.read (argv[1], *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.5);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);
  
  /* dont need the talbe top segmentation
  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);
  */

  //*********************************************************************************
  //********************* Begin Cluster Extraction *********************************
  //*********************************************************************************
 
  // Creating the KdTree object for the search method of the extraction
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    	pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      	  cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    	cloud_cluster->width = cloud_cluster->points.size ();
    	cloud_cluster->height = 1;
    	cloud_cluster->is_dense = true;

    	std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    	std::stringstream ss;
    	ss << "cloud_cluster_" << j << ".pcd";
    	writer.write<PointT> (ss.str (), *cloud_cluster, false); //*

      	//BEGIN SPHERE SEG

	 // Estimate point normals
  	ne.setSearchMethod (tree);
  	ne.setInputCloud (cloud_cluster);
  	ne.setKSearch (50);
  	ne.compute (*cluster_normals);

  	// Create the segmentation object for sphere segmentation and set all the parameters
  	seg.setOptimizeCoefficients (true);
  	seg.setModelType (pcl::SACMODEL_SPHERE);
  	seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setNormalDistanceWeight (0.1);
  	seg.setMaxIterations (10000);
  	seg.setDistanceThreshold (0.05);
  	seg.setRadiusLimits (0, 0.1);
  	seg.setInputCloud (cloud_cluster);
  	seg.setInputNormals (cluster_normals);

  	// Obtain the sphere inliers and coefficients
  	seg.segment (*inliers_sphere, *coefficients_sphere);
  	std::cerr << "Sphere coefficients: " << *coefficients_sphere << std::endl;

	//set coefficients to variables
	if (j==0){
	x0 = coefficients_sphere->values[0];
	y0 = coefficients_sphere->values[1];
	z0 = coefficients_sphere->values[2];
	r0 = coefficients_sphere->values[3];
	}
	if (j==1){
	x1 = coefficients_sphere->values[0];
	y1 = coefficients_sphere->values[1];
	z1 = coefficients_sphere->values[2];
	r1 = coefficients_sphere->values[3];
	}

  	//END SPHERE SEG
   	j++;
  }

  //calulate distance
  d = sqrt(pow((x0-x1),2) + pow((y0-y1),2) + pow((z0-z1),2));
  std::cout << "distance between spheres: " << d << std::endl;

  return (0);
}