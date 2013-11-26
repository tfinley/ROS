cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer_viewports)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_visualizer_picked_segment pcl_visualizer_picked_segment.cpp)
target_link_libraries (pcl_visualizer_picked_segment ${PCL_LIBRARIES})

add_executable (segment_align segment_align.cpp)
target_link_libraries (segment_align ${PCL_LIBRARIES})

add_executable (segment_translate_align segment_translate_align.cpp)
target_link_libraries (segment_translate_align ${PCL_LIBRARIES})

add_executable (segment_downsample_translate_align segment_downsample_translate_align.cpp)
target_link_libraries (segment_downsample_translate_align ${PCL_LIBRARIES})

add_executable (live_segment_downsample_translate_align live_segment_downsample_translate_align.cpp)
target_link_libraries (live_segment_downsample_translate_align ${PCL_LIBRARIES})

add_executable (live_segment_downsample_translate_align_rotate live_segment_downsample_translate_align_rotate.cpp)
target_link_libraries (live_segment_downsample_translate_align_rotate ${PCL_LIBRARIES})

add_executable (live_viewer_start live_viewer_start.cpp)
target_link_libraries (live_viewer_start ${PCL_LIBRARIES})

