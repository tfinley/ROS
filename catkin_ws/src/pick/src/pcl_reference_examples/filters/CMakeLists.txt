cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(filters)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (openni_color_filter openni_color_filter.cpp)
target_link_libraries (openni_color_filter ${PCL_LIBRARIES})



