# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/taylor/src/pcl/examples/segment_align

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/taylor/src/pcl/examples/segment_align/build

# Include any dependencies generated for this target.
include CMakeFiles/live_viewer_start.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/live_viewer_start.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/live_viewer_start.dir/flags.make

CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.o: CMakeFiles/live_viewer_start.dir/flags.make
CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.o: ../live_viewer_start.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/taylor/src/pcl/examples/segment_align/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.o -c /home/taylor/src/pcl/examples/segment_align/live_viewer_start.cpp

CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/taylor/src/pcl/examples/segment_align/live_viewer_start.cpp > CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.i

CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/taylor/src/pcl/examples/segment_align/live_viewer_start.cpp -o CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.s

CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.o.requires:
.PHONY : CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.o.requires

CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.o.provides: CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.o.requires
	$(MAKE) -f CMakeFiles/live_viewer_start.dir/build.make CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.o.provides.build
.PHONY : CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.o.provides

CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.o.provides.build: CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.o

# Object files for target live_viewer_start
live_viewer_start_OBJECTS = \
"CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.o"

# External object files for target live_viewer_start
live_viewer_start_EXTERNAL_OBJECTS =

live_viewer_start: CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.o
live_viewer_start: /usr/lib/libboost_system-mt.so
live_viewer_start: /usr/lib/libboost_filesystem-mt.so
live_viewer_start: /usr/lib/libboost_thread-mt.so
live_viewer_start: /usr/lib/libboost_date_time-mt.so
live_viewer_start: /usr/lib/libboost_iostreams-mt.so
live_viewer_start: /usr/lib/libboost_mpi-mt.so
live_viewer_start: /usr/lib/libboost_serialization-mt.so
live_viewer_start: /usr/local/lib/libpcl_common.so
live_viewer_start: /usr/lib/libflann_cpp_s.a
live_viewer_start: /usr/local/lib/libpcl_kdtree.so
live_viewer_start: /usr/local/lib/libpcl_octree.so
live_viewer_start: /usr/local/lib/libpcl_search.so
live_viewer_start: /usr/lib/libOpenNI.so
live_viewer_start: /usr/lib/libvtkCommon.so.5.8.0
live_viewer_start: /usr/lib/libvtkRendering.so.5.8.0
live_viewer_start: /usr/lib/libvtkHybrid.so.5.8.0
live_viewer_start: /usr/lib/libvtkCharts.so.5.8.0
live_viewer_start: /usr/local/lib/libpcl_io.so
live_viewer_start: /usr/local/lib/libpcl_sample_consensus.so
live_viewer_start: /usr/local/lib/libpcl_filters.so
live_viewer_start: /usr/local/lib/libpcl_features.so
live_viewer_start: /usr/local/lib/libpcl_keypoints.so
live_viewer_start: /usr/local/lib/libpcl_ml.so
live_viewer_start: /usr/local/lib/libpcl_segmentation.so
live_viewer_start: /usr/local/lib/libpcl_visualization.so
live_viewer_start: /usr/local/lib/libpcl_outofcore.so
live_viewer_start: /usr/local/lib/libpcl_stereo.so
live_viewer_start: /usr/lib/libqhull.so
live_viewer_start: /usr/local/lib/libpcl_surface.so
live_viewer_start: /usr/local/lib/libpcl_registration.so
live_viewer_start: /usr/local/lib/libpcl_recognition.so
live_viewer_start: /usr/local/lib/libpcl_tracking.so
live_viewer_start: /usr/local/lib/libpcl_apps.so
live_viewer_start: /usr/local/lib/libpcl_people.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libGLU.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libGL.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libSM.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libICE.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libX11.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libXext.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libGLEW.so
live_viewer_start: /usr/local/lib/libpcl_simulation.so
live_viewer_start: /usr/lib/libboost_system-mt.so
live_viewer_start: /usr/lib/libboost_filesystem-mt.so
live_viewer_start: /usr/lib/libboost_thread-mt.so
live_viewer_start: /usr/lib/libboost_date_time-mt.so
live_viewer_start: /usr/lib/libboost_iostreams-mt.so
live_viewer_start: /usr/lib/libboost_mpi-mt.so
live_viewer_start: /usr/lib/libboost_serialization-mt.so
live_viewer_start: /usr/lib/libqhull.so
live_viewer_start: /usr/lib/libOpenNI.so
live_viewer_start: /usr/lib/libflann_cpp_s.a
live_viewer_start: /usr/lib/libvtkCommon.so.5.8.0
live_viewer_start: /usr/lib/libvtkRendering.so.5.8.0
live_viewer_start: /usr/lib/libvtkHybrid.so.5.8.0
live_viewer_start: /usr/lib/libvtkCharts.so.5.8.0
live_viewer_start: /usr/lib/libvtkViews.so.5.8.0
live_viewer_start: /usr/lib/libvtkInfovis.so.5.8.0
live_viewer_start: /usr/lib/libvtkWidgets.so.5.8.0
live_viewer_start: /usr/lib/libvtkHybrid.so.5.8.0
live_viewer_start: /usr/lib/libvtkParallel.so.5.8.0
live_viewer_start: /usr/lib/libvtkVolumeRendering.so.5.8.0
live_viewer_start: /usr/lib/libvtkRendering.so.5.8.0
live_viewer_start: /usr/lib/libvtkGraphics.so.5.8.0
live_viewer_start: /usr/lib/libvtkImaging.so.5.8.0
live_viewer_start: /usr/lib/libvtkIO.so.5.8.0
live_viewer_start: /usr/lib/libvtkFiltering.so.5.8.0
live_viewer_start: /usr/lib/libvtkCommon.so.5.8.0
live_viewer_start: /usr/lib/libvtksys.so.5.8.0
live_viewer_start: /usr/lib/libboost_system-mt.so
live_viewer_start: /usr/lib/libboost_filesystem-mt.so
live_viewer_start: /usr/lib/libboost_thread-mt.so
live_viewer_start: /usr/lib/libboost_date_time-mt.so
live_viewer_start: /usr/lib/libboost_iostreams-mt.so
live_viewer_start: /usr/lib/libboost_mpi-mt.so
live_viewer_start: /usr/lib/libboost_serialization-mt.so
live_viewer_start: /usr/local/lib/libpcl_common.so
live_viewer_start: /usr/lib/libflann_cpp_s.a
live_viewer_start: /usr/local/lib/libpcl_kdtree.so
live_viewer_start: /usr/local/lib/libpcl_octree.so
live_viewer_start: /usr/local/lib/libpcl_search.so
live_viewer_start: /usr/lib/libOpenNI.so
live_viewer_start: /usr/local/lib/libpcl_io.so
live_viewer_start: /usr/local/lib/libpcl_sample_consensus.so
live_viewer_start: /usr/local/lib/libpcl_filters.so
live_viewer_start: /usr/local/lib/libpcl_features.so
live_viewer_start: /usr/local/lib/libpcl_keypoints.so
live_viewer_start: /usr/local/lib/libpcl_ml.so
live_viewer_start: /usr/local/lib/libpcl_segmentation.so
live_viewer_start: /usr/local/lib/libpcl_visualization.so
live_viewer_start: /usr/local/lib/libpcl_outofcore.so
live_viewer_start: /usr/local/lib/libpcl_stereo.so
live_viewer_start: /usr/lib/libqhull.so
live_viewer_start: /usr/local/lib/libpcl_surface.so
live_viewer_start: /usr/local/lib/libpcl_registration.so
live_viewer_start: /usr/local/lib/libpcl_recognition.so
live_viewer_start: /usr/local/lib/libpcl_tracking.so
live_viewer_start: /usr/local/lib/libpcl_apps.so
live_viewer_start: /usr/local/lib/libpcl_people.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libGLU.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libGL.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libSM.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libICE.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libX11.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libXext.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libGLEW.so
live_viewer_start: /usr/local/lib/libpcl_simulation.so
live_viewer_start: /usr/lib/libboost_system-mt.so
live_viewer_start: /usr/lib/libboost_filesystem-mt.so
live_viewer_start: /usr/lib/libboost_thread-mt.so
live_viewer_start: /usr/lib/libboost_date_time-mt.so
live_viewer_start: /usr/lib/libboost_iostreams-mt.so
live_viewer_start: /usr/lib/libboost_mpi-mt.so
live_viewer_start: /usr/lib/libboost_serialization-mt.so
live_viewer_start: /usr/local/lib/libpcl_common.so
live_viewer_start: /usr/lib/libflann_cpp_s.a
live_viewer_start: /usr/local/lib/libpcl_kdtree.so
live_viewer_start: /usr/local/lib/libpcl_octree.so
live_viewer_start: /usr/local/lib/libpcl_search.so
live_viewer_start: /usr/lib/libOpenNI.so
live_viewer_start: /usr/local/lib/libpcl_io.so
live_viewer_start: /usr/local/lib/libpcl_sample_consensus.so
live_viewer_start: /usr/local/lib/libpcl_filters.so
live_viewer_start: /usr/local/lib/libpcl_features.so
live_viewer_start: /usr/local/lib/libpcl_keypoints.so
live_viewer_start: /usr/local/lib/libpcl_ml.so
live_viewer_start: /usr/local/lib/libpcl_segmentation.so
live_viewer_start: /usr/local/lib/libpcl_visualization.so
live_viewer_start: /usr/local/lib/libpcl_outofcore.so
live_viewer_start: /usr/local/lib/libpcl_stereo.so
live_viewer_start: /usr/lib/libqhull.so
live_viewer_start: /usr/local/lib/libpcl_surface.so
live_viewer_start: /usr/local/lib/libpcl_registration.so
live_viewer_start: /usr/local/lib/libpcl_recognition.so
live_viewer_start: /usr/local/lib/libpcl_tracking.so
live_viewer_start: /usr/local/lib/libpcl_apps.so
live_viewer_start: /usr/local/lib/libpcl_people.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libGLU.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libGL.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libSM.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libICE.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libX11.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libXext.so
live_viewer_start: /usr/lib/x86_64-linux-gnu/libGLEW.so
live_viewer_start: /usr/local/lib/libpcl_simulation.so
live_viewer_start: CMakeFiles/live_viewer_start.dir/build.make
live_viewer_start: CMakeFiles/live_viewer_start.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable live_viewer_start"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/live_viewer_start.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/live_viewer_start.dir/build: live_viewer_start
.PHONY : CMakeFiles/live_viewer_start.dir/build

CMakeFiles/live_viewer_start.dir/requires: CMakeFiles/live_viewer_start.dir/live_viewer_start.cpp.o.requires
.PHONY : CMakeFiles/live_viewer_start.dir/requires

CMakeFiles/live_viewer_start.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/live_viewer_start.dir/cmake_clean.cmake
.PHONY : CMakeFiles/live_viewer_start.dir/clean

CMakeFiles/live_viewer_start.dir/depend:
	cd /home/taylor/src/pcl/examples/segment_align/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/taylor/src/pcl/examples/segment_align /home/taylor/src/pcl/examples/segment_align /home/taylor/src/pcl/examples/segment_align/build /home/taylor/src/pcl/examples/segment_align/build /home/taylor/src/pcl/examples/segment_align/build/CMakeFiles/live_viewer_start.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/live_viewer_start.dir/depend

