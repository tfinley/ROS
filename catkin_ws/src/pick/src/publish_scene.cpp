#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"
#include <boost/foreach.hpp>
#include <sstream>
#include <ros/package.h> 

int main(int argc, char **argv) {
	ros::init(argc, argv, "publish_scene");

	ros::NodeHandle n;

	ros::Publisher d_info_pub = n.advertise<sensor_msgs::CameraInfo>("study/depth/camera_info", 1);
	ros::Publisher d_points_pub = n.advertise<sensor_msgs::PointCloud2>("study/depth/points", 1);
	ros::Publisher rgb_info_pub = n.advertise<sensor_msgs::CameraInfo>("study/rgb/camera_info", 1);
	ros::Publisher rgb_image_pub = n.advertise<sensor_msgs::Image>("study/rgb/image", 1);
	ros::Rate loop_rate(10);

  std::string package_path = ros::package::getPath("pick");
  std::string scene_path;

  while (ros::ok()){

    int scene_number;
    if (n.getParam("/study/scene_number", scene_number))
    {
      std::stringstream sstm;
      if (scene_number > 9)
        sstm << package_path << "/scenes/scene_" << scene_number << ".bag";
      else
        sstm << package_path << "/scenes/scene_0" << scene_number << ".bag";
      
      scene_path = sstm.str();
    }
    else
    {
      std::cout << "rosparam /study/scene number not set" << std::endl;
      continue;
    }

  	rosbag::Bag bag;
    bag.open(scene_path, rosbag::bagmode::Read);

  	std::string depth_cam_info = "/camera/depth_registered/camera_info";
    std::string depth_points = "/camera/depth_registered/points";
    std::string rgb_cam_info = "/camera/rgb/camera_info";
    std::string rgb_cam_image = "/camera/rgb/image_color";

    std::vector<std::string> topics;
    topics.push_back(depth_cam_info);
    topics.push_back(depth_points);
    topics.push_back(rgb_cam_info);
    topics.push_back(rgb_cam_image);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      if (m.getTopic() == depth_cam_info) 
      {
        sensor_msgs::CameraInfo::ConstPtr d_info_ptr = m.instantiate<sensor_msgs::CameraInfo>();
        if (d_info_ptr != NULL)
          d_info_pub.publish(*d_info_ptr);
      }
        
      if (m.getTopic() == depth_points) 
      {
        sensor_msgs::PointCloud2::ConstPtr d_points_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        if (d_points_ptr != NULL)
          d_points_pub.publish(*d_points_ptr);
      }
        
      if (m.getTopic() == rgb_cam_info) 
      {
        sensor_msgs::CameraInfo::ConstPtr rgb_info_ptr = m.instantiate<sensor_msgs::CameraInfo>();
        if (rgb_info_ptr != NULL)
          rgb_info_pub.publish(*rgb_info_ptr);
      }
      
      if (m.getTopic() == rgb_cam_image) 
      {
        sensor_msgs::Image::ConstPtr rgb_image_ptr = m.instantiate<sensor_msgs::Image>();
        if (rgb_image_ptr != NULL)
          rgb_image_pub.publish(*rgb_image_ptr);
      }

    }

  bag.close();
  ros::spinOnce();
  loop_rate.sleep();

  }

}