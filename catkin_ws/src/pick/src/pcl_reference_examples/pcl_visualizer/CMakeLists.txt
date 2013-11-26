cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer_viewports)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_visualizer_demo pcl_visualizer_demo.cpp)
target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES})

add_executable (pcl_visualizer_kinect pcl_visualizer_kinect.cpp)
target_link_libraries (pcl_visualizer_kinect ${PCL_LIBRARIES})

add_executable (pcl_visualizer_crop pcl_visualizer_crop.cpp)
target_link_libraries (pcl_visualizer_crop ${PCL_LIBRARIES})

add_executable (pcl_visualizer_passthrough pcl_visualizer_passthrough.cpp)
target_link_libraries (pcl_visualizer_passthrough ${PCL_LIBRARIES})

add_executable (pcl_visualizer_stripped_down pcl_visualizer_stripped_down.cpp)
target_link_libraries (pcl_visualizer_stripped_down ${PCL_LIBRARIES})

add_executable (pcl_visualizer_picked_segment pcl_visualizer_picked_segment.cpp)
target_link_libraries (pcl_visualizer_picked_segment ${PCL_LIBRARIES})

add_executable (pcl_visualizer_picked_rgb pcl_visualizer_picked_rgb.cpp)
target_link_libraries (pcl_visualizer_picked_rgb ${PCL_LIBRARIES})
