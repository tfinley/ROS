#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h> 
#include <sstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_instruction_image");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/study/rgb/instructions", 1);

  ros::Rate loop_rate(10);

  while (nh.ok()) 
  {
    //Get part number from paramer server
    int instruction_number;
    nh.getParam("/study/instruction_number", instruction_number);

    //Setup image file path
    std::string package_path = ros::package::getPath("pick");  
    std::string file_path ;
    std::stringstream sstm;
    sstm << package_path << "/instruction_images/" << instruction_number << ".jpeg";
    file_path = sstm.str();

    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread(file_path,CV_LOAD_IMAGE_COLOR);
    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);

    //adding this to test adding frame id to work in RVIZ
    std_msgs::Header img_header;
    img_header.frame_id = "/camera_rgb_optical_frame";
    ros_image.header = img_header;
    //end of test

    pub.publish(ros_image);
    loop_rate.sleep();
  }
}




