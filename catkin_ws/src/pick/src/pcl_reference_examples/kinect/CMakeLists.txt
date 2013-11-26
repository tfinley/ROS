cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer_viewports)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (kinect_viewer kinect_viewer.cpp)
target_link_libraries (kinect_viewer ${PCL_LIBRARIES})

add_executable (kinect_visualizer kinect_visualizer.cpp)
target_link_libraries (kinect_visualizer ${PCL_LIBRARIES})

add_executable (kinect_visualizer2 kinect_visualizer2.cpp)
target_link_libraries (kinect_visualizer2 ${PCL_LIBRARIES})

add_executable (kinect_viewer_stripped kinect_viewer_stripped.cpp)
target_link_libraries (kinect_viewer_stripped ${PCL_LIBRARIES})

add_executable (openni_save_pcd openni_save_pcd.cpp)
target_link_libraries (openni_save_pcd ${PCL_LIBRARIES})

