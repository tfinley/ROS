/*

Originally created by Taylor Finley, 2013.11.21
License: BSD
Description: 	This will manually create a transformation matrix
similar to the output of ICP. It will then do the proper conversions
to be able to publish out the center tf of the lifting eye.

*/

#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>


using namespace std;



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "transform_test");
  ros::NodeHandle nh;
  static tf::TransformBroadcaster br;

  // Create the transform matrix between the origin and the center of the eye (when imported)
  // NOTE:  when imported the origin is at the top of the lifting eye
  // This is to help when the lifting eye is translated to the picked point (which will probably
  // be at the top of the lifting eye.
  Eigen::Matrix4f centerOffset;
  centerOffset << 	1, 0, 0, 0,
		 	0, 1, 0, 0,
			0, 0, 1, 0.027,
			0, 0, 0, 1;
 
  Eigen::Matrix4f Tm;
  Tm << 	0.714191,   -0.54278,   0.441988,  0.0432322,
  		0.409255,   0.836069,    0.36542,  0.0571429,
 		-0.567861, -0.0800918,   0.819232,    1.22178,
         	0,          0,          0,	     1; 

  // copy the original transform in case it is needed to transform the cloud later.
  Eigen::Matrix4f originalTm = Tm;
  // calculate center of lifting eye tranform
  Tm = centerOffset * Tm;

  tf::Vector3 origin;
  origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));

  cout << origin << endl;
  tf::Matrix3x3 tf3d;
  tf3d.setValue(static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)), 
		static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)), static_cast<double>(Tm(1,2)), 
		static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)), static_cast<double>(Tm(2,2)));

  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);

  tf::Transform transform;
  transform.setOrigin(origin);
  transform.setRotation(tfqt);
  while (true) br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_rgb_optical_frame", "lifting_eye"));

        

  // Spin
  ros::spin ();
}
