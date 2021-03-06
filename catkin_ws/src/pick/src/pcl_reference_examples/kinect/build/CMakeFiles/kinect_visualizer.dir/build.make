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
CMAKE_SOURCE_DIR = /home/taylor/src/pcl/examples/kinect

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/taylor/src/pcl/examples/kinect/build

# Include any dependencies generated for this target.
include CMakeFiles/kinect_visualizer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/kinect_visualizer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/kinect_visualizer.dir/flags.make

CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.o: CMakeFiles/kinect_visualizer.dir/flags.make
CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.o: ../kinect_visualizer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/taylor/src/pcl/examples/kinect/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.o -c /home/taylor/src/pcl/examples/kinect/kinect_visualizer.cpp

CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/taylor/src/pcl/examples/kinect/kinect_visualizer.cpp > CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.i

CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/taylor/src/pcl/examples/kinect/kinect_visualizer.cpp -o CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.s

CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.o.requires:
.PHONY : CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.o.requires

CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.o.provides: CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.o.requires
	$(MAKE) -f CMakeFiles/kinect_visualizer.dir/build.make CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.o.provides.build
.PHONY : CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.o.provides

CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.o.provides.build: CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.o

# Object files for target kinect_visualizer
kinect_visualizer_OBJECTS = \
"CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.o"

# External object files for target kinect_visualizer
kinect_visualizer_EXTERNAL_OBJECTS =

kinect_visualizer: CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.o
kinect_visualizer: /usr/lib/libboost_system-mt.so
kinect_visualizer: /usr/lib/libboost_filesystem-mt.so
kinect_visualizer: /usr/lib/libboost_thread-mt.so
kinect_visualizer: /usr/lib/libboost_date_time-mt.so
kinect_visualizer: /usr/lib/libboost_iostreams-mt.so
kinect_visualizer: /usr/lib/libboost_mpi-mt.so
kinect_visualizer: /usr/lib/libboost_serialization-mt.so
kinect_visualizer: /usr/local/lib/libpcl_common.so
kinect_visualizer: /usr/lib/libflann_cpp_s.a
kinect_visualizer: /usr/local/lib/libpcl_kdtree.so
kinect_visualizer: /usr/local/lib/libpcl_octree.so
kinect_visualizer: /usr/local/lib/libpcl_search.so
kinect_visualizer: /usr/lib/libOpenNI.so
kinect_visualizer: /usr/lib/libvtkCommon.so.5.8.0
kinect_visualizer: /usr/lib/libvtkRendering.so.5.8.0
kinect_visualizer: /usr/lib/libvtkHybrid.so.5.8.0
kinect_visualizer: /usr/lib/libvtkCharts.so.5.8.0
kinect_visualizer: /usr/local/lib/libpcl_io.so
kinect_visualizer: /usr/local/lib/libpcl_sample_consensus.so
kinect_visualizer: /usr/local/lib/libpcl_filters.so
kinect_visualizer: /usr/local/lib/libpcl_features.so
kinect_visualizer: /usr/local/lib/libpcl_keypoints.so
kinect_visualizer: /usr/local/lib/libpcl_ml.so
kinect_visualizer: /usr/local/lib/libpcl_segmentation.so
kinect_visualizer: /usr/local/lib/libpcl_visualization.so
kinect_visualizer: /usr/local/lib/libpcl_outofcore.so
kinect_visualizer: /usr/local/lib/libpcl_stereo.so
kinect_visualizer: /usr/lib/libqhull.so
kinect_visualizer: /usr/local/lib/libpcl_surface.so
kinect_visualizer: /usr/local/lib/libpcl_registration.so
kinect_visualizer: /usr/local/lib/libpcl_recognition.so
kinect_visualizer: /usr/local/lib/libpcl_tracking.so
kinect_visualizer: /usr/local/lib/libpcl_apps.so
kinect_visualizer: /usr/local/lib/libpcl_people.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libGLU.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libGL.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libSM.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libICE.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libX11.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libXext.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libGLEW.so
kinect_visualizer: /usr/local/lib/libpcl_simulation.so
kinect_visualizer: /usr/lib/libboost_system-mt.so
kinect_visualizer: /usr/lib/libboost_filesystem-mt.so
kinect_visualizer: /usr/lib/libboost_thread-mt.so
kinect_visualizer: /usr/lib/libboost_date_time-mt.so
kinect_visualizer: /usr/lib/libboost_iostreams-mt.so
kinect_visualizer: /usr/lib/libboost_mpi-mt.so
kinect_visualizer: /usr/lib/libboost_serialization-mt.so
kinect_visualizer: /usr/lib/libqhull.so
kinect_visualizer: /usr/lib/libOpenNI.so
kinect_visualizer: /usr/lib/libflann_cpp_s.a
kinect_visualizer: /usr/lib/libvtkCommon.so.5.8.0
kinect_visualizer: /usr/lib/libvtkRendering.so.5.8.0
kinect_visualizer: /usr/lib/libvtkHybrid.so.5.8.0
kinect_visualizer: /usr/lib/libvtkCharts.so.5.8.0
kinect_visualizer: /usr/lib/libvtkViews.so.5.8.0
kinect_visualizer: /usr/lib/libvtkInfovis.so.5.8.0
kinect_visualizer: /usr/lib/libvtkWidgets.so.5.8.0
kinect_visualizer: /usr/lib/libvtkHybrid.so.5.8.0
kinect_visualizer: /usr/lib/libvtkParallel.so.5.8.0
kinect_visualizer: /usr/lib/libvtkVolumeRendering.so.5.8.0
kinect_visualizer: /usr/lib/libvtkRendering.so.5.8.0
kinect_visualizer: /usr/lib/libvtkGraphics.so.5.8.0
kinect_visualizer: /usr/lib/libvtkImaging.so.5.8.0
kinect_visualizer: /usr/lib/libvtkIO.so.5.8.0
kinect_visualizer: /usr/lib/libvtkFiltering.so.5.8.0
kinect_visualizer: /usr/lib/libvtkCommon.so.5.8.0
kinect_visualizer: /usr/lib/libvtksys.so.5.8.0
kinect_visualizer: /usr/lib/libboost_system-mt.so
kinect_visualizer: /usr/lib/libboost_filesystem-mt.so
kinect_visualizer: /usr/lib/libboost_thread-mt.so
kinect_visualizer: /usr/lib/libboost_date_time-mt.so
kinect_visualizer: /usr/lib/libboost_iostreams-mt.so
kinect_visualizer: /usr/lib/libboost_mpi-mt.so
kinect_visualizer: /usr/lib/libboost_serialization-mt.so
kinect_visualizer: /usr/local/lib/libpcl_common.so
kinect_visualizer: /usr/lib/libflann_cpp_s.a
kinect_visualizer: /usr/local/lib/libpcl_kdtree.so
kinect_visualizer: /usr/local/lib/libpcl_octree.so
kinect_visualizer: /usr/local/lib/libpcl_search.so
kinect_visualizer: /usr/lib/libOpenNI.so
kinect_visualizer: /usr/local/lib/libpcl_io.so
kinect_visualizer: /usr/local/lib/libpcl_sample_consensus.so
kinect_visualizer: /usr/local/lib/libpcl_filters.so
kinect_visualizer: /usr/local/lib/libpcl_features.so
kinect_visualizer: /usr/local/lib/libpcl_keypoints.so
kinect_visualizer: /usr/local/lib/libpcl_ml.so
kinect_visualizer: /usr/local/lib/libpcl_segmentation.so
kinect_visualizer: /usr/local/lib/libpcl_visualization.so
kinect_visualizer: /usr/local/lib/libpcl_outofcore.so
kinect_visualizer: /usr/local/lib/libpcl_stereo.so
kinect_visualizer: /usr/lib/libqhull.so
kinect_visualizer: /usr/local/lib/libpcl_surface.so
kinect_visualizer: /usr/local/lib/libpcl_registration.so
kinect_visualizer: /usr/local/lib/libpcl_recognition.so
kinect_visualizer: /usr/local/lib/libpcl_tracking.so
kinect_visualizer: /usr/local/lib/libpcl_apps.so
kinect_visualizer: /usr/local/lib/libpcl_people.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libGLU.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libGL.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libSM.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libICE.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libX11.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libXext.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libGLEW.so
kinect_visualizer: /usr/local/lib/libpcl_simulation.so
kinect_visualizer: /usr/lib/libboost_system-mt.so
kinect_visualizer: /usr/lib/libboost_filesystem-mt.so
kinect_visualizer: /usr/lib/libboost_thread-mt.so
kinect_visualizer: /usr/lib/libboost_date_time-mt.so
kinect_visualizer: /usr/lib/libboost_iostreams-mt.so
kinect_visualizer: /usr/lib/libboost_mpi-mt.so
kinect_visualizer: /usr/lib/libboost_serialization-mt.so
kinect_visualizer: /usr/local/lib/libpcl_common.so
kinect_visualizer: /usr/lib/libflann_cpp_s.a
kinect_visualizer: /usr/local/lib/libpcl_kdtree.so
kinect_visualizer: /usr/local/lib/libpcl_octree.so
kinect_visualizer: /usr/local/lib/libpcl_search.so
kinect_visualizer: /usr/lib/libOpenNI.so
kinect_visualizer: /usr/local/lib/libpcl_io.so
kinect_visualizer: /usr/local/lib/libpcl_sample_consensus.so
kinect_visualizer: /usr/local/lib/libpcl_filters.so
kinect_visualizer: /usr/local/lib/libpcl_features.so
kinect_visualizer: /usr/local/lib/libpcl_keypoints.so
kinect_visualizer: /usr/local/lib/libpcl_ml.so
kinect_visualizer: /usr/local/lib/libpcl_segmentation.so
kinect_visualizer: /usr/local/lib/libpcl_visualization.so
kinect_visualizer: /usr/local/lib/libpcl_outofcore.so
kinect_visualizer: /usr/local/lib/libpcl_stereo.so
kinect_visualizer: /usr/lib/libqhull.so
kinect_visualizer: /usr/local/lib/libpcl_surface.so
kinect_visualizer: /usr/local/lib/libpcl_registration.so
kinect_visualizer: /usr/local/lib/libpcl_recognition.so
kinect_visualizer: /usr/local/lib/libpcl_tracking.so
kinect_visualizer: /usr/local/lib/libpcl_apps.so
kinect_visualizer: /usr/local/lib/libpcl_people.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libGLU.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libGL.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libSM.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libICE.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libX11.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libXext.so
kinect_visualizer: /usr/lib/x86_64-linux-gnu/libGLEW.so
kinect_visualizer: /usr/local/lib/libpcl_simulation.so
kinect_visualizer: CMakeFiles/kinect_visualizer.dir/build.make
kinect_visualizer: CMakeFiles/kinect_visualizer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable kinect_visualizer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kinect_visualizer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/kinect_visualizer.dir/build: kinect_visualizer
.PHONY : CMakeFiles/kinect_visualizer.dir/build

CMakeFiles/kinect_visualizer.dir/requires: CMakeFiles/kinect_visualizer.dir/kinect_visualizer.cpp.o.requires
.PHONY : CMakeFiles/kinect_visualizer.dir/requires

CMakeFiles/kinect_visualizer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/kinect_visualizer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/kinect_visualizer.dir/clean

CMakeFiles/kinect_visualizer.dir/depend:
	cd /home/taylor/src/pcl/examples/kinect/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/taylor/src/pcl/examples/kinect /home/taylor/src/pcl/examples/kinect /home/taylor/src/pcl/examples/kinect/build /home/taylor/src/pcl/examples/kinect/build /home/taylor/src/pcl/examples/kinect/build/CMakeFiles/kinect_visualizer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/kinect_visualizer.dir/depend

