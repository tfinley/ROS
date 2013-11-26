cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(conditional_removal_color)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (conditional_removal_color conditional_removal_color.cpp)
target_link_libraries (conditional_removal_color ${PCL_LIBRARIES})

add_executable (conditional_removal_color_outlier conditional_removal_color_outlier.cpp)
target_link_libraries (conditional_removal_color_outlier ${PCL_LIBRARIES})
