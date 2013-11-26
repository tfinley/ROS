cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(iterative_closest_point)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (iterative_closest_point iterative_closest_point.cpp)
target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})

add_executable (iterative_closest_point_with_input iterative_closest_point_with_input.cpp)
target_link_libraries (iterative_closest_point_with_input ${PCL_LIBRARIES})

add_executable (iterative_closest_point_single iterative_closest_point_single.cpp)
target_link_libraries (iterative_closest_point_single ${PCL_LIBRARIES})

add_executable (iterative_closest_point_nl iterative_closest_point_nl.cpp)
target_link_libraries (iterative_closest_point_nl ${PCL_LIBRARIES})

