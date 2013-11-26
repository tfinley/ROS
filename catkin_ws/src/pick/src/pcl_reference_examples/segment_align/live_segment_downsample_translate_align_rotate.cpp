/* 

original authors
---------------------------------------------------
PCLVisualizer: Geoffrey Biggs 
Pairwise_incremental_registration: Radu Bogdan Susu
iterative_closest_point: PCL
---------------------------------------------------
customized: Taylor Finley

Description: The kinect will stream to the viewer. The user will hit the space bar to save the cloud. The target cloud will be loaded into the PCLVisualizer. 
The user will pick the point (holding the shift key) the data will be segmented around the pick point. The scene will then be downsampled. The liftnig eye will be translated to the picked point (this is the key difference bewtween the "segment_align" Then. program ICP will attempt to align the source (lifting eye) to the segmented cloud.

Call: ./segment_downsample_transform_align_live 

The source will be the reference point cloud (liftnig eye) and the target will be the scene from the kinect.

*/


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
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

//global variables
//visualizer
pcl::visualization::PCLVisualizer *viewer;
//x,y,z values for picked point
float picked_x, picked_y, picked_z;
bool pointPicked = false;
bool translated = false;
bool segmented = false;
Eigen::Matrix4f Tm; //Transformation Matrix
Eigen::Matrix4f Tm180; //Rotation Matrix to rotate the lifting eye
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line_transformed (new pcl::PointCloud<pcl::PointXYZ>);



//TF Customization added from example to print out xyz from picking of point from cloud

void pp_callback (const pcl::visualization::PointPickingEvent &event) 
{ 
  
  //float picked_x, picked_y, picked_z;
  if (event.getPointIndex () == -1)
  {
    cout << "pointindex = -1" << endl;
    return;
  }
  event.getPoint(picked_x, picked_y, picked_z);
  //I'm getting a lot of missed points and segmenting in the wrong area. this will help prevent this.s
  if (picked_z < 0.1)
  {
    cout << "missed point: " << picked_z << endl;
    return;
  }
  cout << "picked point: " << picked_x << "  " << picked_y << " " << picked_z << endl;
  pointPicked = true;
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  if (event.getKeySym () == "z" && event.keyDown ())
  {
	
	cout << "flipping orientation..." << endl;
	//calculate final transform with flip
	Tm = Tm * Tm180;
	cout << "Tm after flip: " << endl;
	cout << Tm << endl;

	//Flipping cloud and final transform
	pcl::transformPointCloud (*cloud_line, *cloud_line_transformed, Tm);

	//remove line
	viewer->removeShape("line", 0);
	viewer->spinOnce(100);

	//add line 
	viewer->addLine<pcl::PointXYZ> (cloud_line_transformed->points[0],cloud_line_transformed->points[1], 1.0,0.0,0.0, "line");
	viewer->spinOnce(100);
	cout << "flipped"  << endl;
	
	/*
	//trying to draw a cone
	//get the two points to define the cone
	pcl::PointXYZ conePt0, conePt1;
	conePt0 = cloud_line_transformed->points[0];
	conePt1 = cloud_line_transformed->points[1];
	//conePt1 = conePt1 - conePt0;

	pcl::ModelCoefficients cone_coeff;
        cone_coeff.values.resize (7);    // We need 7 values
        cone_coeff.values[0] = conePt0.x ; 
        cone_coeff.values[1] = conePt0.y ;
        cone_coeff.values[2] = conePt0.z ;

        cone_coeff.values[3] = conePt1.x - conePt0.x;
        cone_coeff.values[4] = conePt1.y - conePt0.y;
        cone_coeff.values[5] = conePt1.z - conePt0.z;
  	cout << "x: " << cone_coeff.values[3] << " y: " << cone_coeff.values[4] <<  " z: " << cone_coeff.values[5] << endl;       
        cone_coeff.values[6] = 0.1;
	
	viewer->removeShape("Center", 0);
 	viewer->addSphere(conePt0, 0.005, 0.0, 0.0, 1.0, "Cone1", 0);
 	//viewer->addSphere(conePt1, 0.005, 0.0, 0.0, 1.0, "Cone2", 0);

	viewer->addCone(cone_coeff, "cone", 0);
	*/

  }
}



// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{ 
  
  /* Initial segmentation test load of pcd file
  // TF---Load PCD file from Kinect 
  // I really dont know how the creation of the pointer and point cloud work here
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr_kinect (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>& point_cloud_kinect = *point_cloud_ptr_kinect;
  pcl::io::loadPCDFile ("/home/taylor/src/data_pcd/top/kinect_top_rgb.pcd", point_cloud_kinect);
  */

  //setup
  
  //create rotation matrix
  Tm180 << 	-1, 0, 0, 0,
		0, -1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;


  //load pcd files from arguments
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_translated (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile ("/home/taylor/src/data_pcd/top/lifting_eye_aligned_m.pcd", *cloud_source) < 0)
  {
    std::cout << "Error loading model cloud." << std::endl;
    return (-1);
  }
  if (pcl::io::loadPCDFile ("liveOutput.pcd", *cloud_target) < 0)
  {
    std::cout << "Error loading scene cloud." << std::endl;
    return (-1);
  }

  // -----Open 3D viewer and add point cloud-----
  viewer = new pcl::visualization::PCLVisualizer ("3D Viewer");
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_target); //what does this do?
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud_target, rgb, "initial cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "initial cloud");
  viewer->registerPointPickingCallback (pp_callback);
  viewer->registerKeyboardCallback (keyboardEventOccurred); //was   //viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);

  //TF custom camera position for this point cloud from Kinect to make it look like a 2D picture
  viewer->spinOnce(1000);
  viewer->setCameraPosition(0.00, 0.00, -1.25, 0.00, 0.00, 0.625, -0.00, -0.99999, 0.000);
  viewer->setCameraFieldOfView(0.523599);
  viewer->setCameraClipDistances(0.0, 4.0);
  viewer->setSize(800,800);
  viewer->spinOnce(100);
  cout << " " << endl;
   
  //create new clouds to save the filtered
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xy (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xyz (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);


  //define points for line (arrow)
  pcl::PointXYZ centerPt;
  pcl::PointXYZ lineEndPt;

  centerPt.x = 0.00;
  centerPt.y = 0.00;
  centerPt.z = 0.0273;
  cloud_line->push_back(centerPt);
  lineEndPt.x = 0.00;
  lineEndPt.y = 0.100;
  lineEndPt.z = 0.0273;
  cloud_line->push_back(lineEndPt);
  cout << "done" << endl;

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    //
    if (pointPicked)
    {
    
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

   	std::cout << "Cloud before filtering: " << cloud_target->points.size() << std::endl;


  
  	// Create the filtering object x
  	pcl::PassThrough<pcl::PointXYZRGB> pass_x;
  	pass_x.setInputCloud (cloud_target);
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

	//start downsample
	// Create the filtering object
  	pcl::VoxelGrid<pcl::PointXYZRGB> vox;
  	vox.setInputCloud (cloud_filtered_xyz);
  	vox.setLeafSize (0.002f, 0.002f, 0.002f);
  	vox.filter (*cloud_filtered_downsampled);
	std::cout << "Cloud after downsampling: " << cloud_filtered_downsampled->points.size() << std::endl;  
	
	//turn off point picked bool
	pointPicked = false;

	//turn on segmented bool
	segmented = true;
    }
    
    //translate source to picked point
    if (segmented)
    {

	Tm << 	1, 0, 0, picked_x,
		0, 1, 0, picked_y,
		0, 0, 1, picked_z,
		0, 0, 0, 1;
	pcl::transformPointCloud(*cloud_source, *cloud_source_translated, Tm);
	segmented = false;
	translated = true;

    }

    //icp
    if (translated)
    {
	
	//setup ICP
  	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  	icp.setEuclideanFitnessEpsilon (1e-10);
	icp.setTransformationEpsilon (1e-6);
  	icp.setMaxCorrespondenceDistance (.1);
  	icp.setMaximumIterations(50);
  	icp.setInputSource(cloud_source_translated);
  	icp.setInputTarget(cloud_filtered_downsampled);

  	//copy the source cloud
  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_aligned = cloud_source_translated;
	Eigen::Matrix4f prev;
 
  	//Forced iteractions with ICP loop
  	for (int i = 0; i < 10; ++i)
  	{
    	  PCL_INFO ("Iteration Nr. %d.\n", i);
    	  cloud_source_translated = cloud_source_aligned; //when you set to pointers equal does it actaully set the point clouds equal?
    	  icp.setInputSource (cloud_source_translated);
    	  icp.align (*cloud_source_aligned);
	  Tm = icp.getFinalTransformation () * Tm;
    	  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	  //added change to each iteration
	  if (fabs ((icp.getLastIncrementalTransformation () - prev).sum ()) < icp.getTransformationEpsilon ())
      		icp.setMaxCorrespondenceDistance (icp.getMaxCorrespondenceDistance () - 0.001);
    
    	  prev = icp.getLastIncrementalTransformation ();
  	}
  	std::cout << "Final Tm:" << std::endl;
  	std::cout << Tm << std::endl;

	//transform center of lifing eye line (cloud of 2 points)
	pcl::transformPointCloud (*cloud_line, *cloud_line_transformed, Tm);

  	//add sphere to center of lifting eye
 	viewer->addSphere(cloud_line_transformed->points[0], 0.005, 1.0, 0.0, 0.0, "Center", 0);

	//add line 
	viewer->addLine<pcl::PointXYZ> (cloud_line_transformed->points[0],cloud_line_transformed->points[1], 1.0,0.0,0.0, "line");

	/*
	//add arrow PLY file
	vtkSmartPointer<vtkTransform> Tm_Ptr = Tm; //pointer to transform
	//*Tm_Ptr = Tm;
	viewer->addModelFromPLYFile("/home/taylor/src/data_pcd/top/arrow.ply", "PolyData", 0);
	*/

  	pcl::io::savePCDFileBinary ("output.pcd", *cloud_source_aligned);
  	pcl::io::savePCDFileBinary ("scene_downsampled.pcd", *cloud_filtered_downsampled);

	//add new cloud to viewer
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud_source_aligned, 0, 255, 0);
  	viewer->addPointCloud<pcl::PointXYZRGB> (cloud_source_aligned, single_color, "aligned cloud");
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "aligned cloud");
 
	//turn off segmented
	translated = false;
    }

 
  }
}
