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
include CMakeFiles/segment_translate_align.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/segment_translate_align.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/segment_translate_align.dir/flags.make

CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.o: CMakeFiles/segment_translate_align.dir/flags.make
CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.o: ../segment_translate_align.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/taylor/src/pcl/examples/segment_align/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.o -c /home/taylor/src/pcl/examples/segment_align/segment_translate_align.cpp

CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/taylor/src/pcl/examples/segment_align/segment_translate_align.cpp > CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.i

CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/taylor/src/pcl/examples/segment_align/segment_translate_align.cpp -o CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.s

CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.o.requires:
.PHONY : CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.o.requires

CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.o.provides: CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.o.requires
	$(MAKE) -f CMakeFiles/segment_translate_align.dir/build.make CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.o.provides.build
.PHONY : CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.o.provides

CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.o.provides.build: CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.o

# Object files for target segment_translate_align
segment_translate_align_OBJECTS = \
"CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.o"

# External object files for target segment_translate_align
segment_translate_align_EXTERNAL_OBJECTS =

segment_translate_align: CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.o
segment_translate_align: /usr/lib/libboost_system-mt.so
segment_translate_align: /usr/lib/libboost_filesystem-mt.so
segment_translate_align: /usr/lib/libboost_thread-mt.so
segment_translate_align: /usr/lib/libboost_date_time-mt.so
segment_translate_align: /usr/lib/libboost_iostreams-mt.so
segment_translate_align: /usr/lib/libboost_mpi-mt.so
segment_translate_align: /usr/lib/libboost_serialization-mt.so
segment_translate_align: /usr/local/lib/libpcl_common.so
segment_translate_align: /usr/lib/libflann_cpp_s.a
segment_translate_align: /usr/local/lib/libpcl_kdtree.so
segment_translate_align: /usr/local/lib/libpcl_octree.so
segment_translate_align: /usr/local/lib/libpcl_search.so
segment_translate_align: /usr/lib/libOpenNI.so
segment_translate_align: /usr/lib/libvtkCommon.so.5.8.0
segment_translate_align: /usr/lib/libvtkRendering.so.5.8.0
segment_translate_align: /usr/lib/libvtkHybrid.so.5.8.0
segment_translate_align: /usr/lib/libvtkCharts.so.5.8.0
segment_translate_align: /usr/local/lib/libpcl_io.so
segment_translate_align: /usr/local/lib/libpcl_sample_consensus.so
segment_translate_align: /usr/local/lib/libpcl_filters.so
segment_translate_align: /usr/local/lib/libpcl_features.so
segment_translate_align: /usr/local/lib/libpcl_keypoints.so
segment_translate_align: /usr/local/lib/libpcl_ml.so
segment_translate_align: /usr/local/lib/libpcl_segmentation.so
segment_translate_align: /usr/local/lib/libpcl_visualization.so
segment_translate_align: /usr/local/lib/libpcl_outofcore.so
segment_translate_align: /usr/local/lib/libpcl_stereo.so
segment_translate_align: /usr/lib/libqhull.so
segment_translate_align: /usr/local/lib/libpcl_surface.so
segment_translate_align: /usr/local/lib/libpcl_registration.so
segment_translate_align: /usr/local/lib/libpcl_recognition.so
segment_translate_align: /usr/local/lib/libpcl_tracking.so
segment_translate_align: /usr/local/lib/libpcl_apps.so
segment_translate_align: /usr/local/lib/libpcl_people.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libGLU.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libGL.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libSM.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libICE.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libX11.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libXext.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libGLEW.so
segment_translate_align: /usr/local/lib/libpcl_simulation.so
segment_translate_align: /usr/lib/libboost_system-mt.so
segment_translate_align: /usr/lib/libboost_filesystem-mt.so
segment_translate_align: /usr/lib/libboost_thread-mt.so
segment_translate_align: /usr/lib/libboost_date_time-mt.so
segment_translate_align: /usr/lib/libboost_iostreams-mt.so
segment_translate_align: /usr/lib/libboost_mpi-mt.so
segment_translate_align: /usr/lib/libboost_serialization-mt.so
segment_translate_align: /usr/lib/libqhull.so
segment_translate_align: /usr/lib/libOpenNI.so
segment_translate_align: /usr/lib/libflann_cpp_s.a
segment_translate_align: /usr/lib/libvtkCommon.so.5.8.0
segment_translate_align: /usr/lib/libvtkRendering.so.5.8.0
segment_translate_align: /usr/lib/libvtkHybrid.so.5.8.0
segment_translate_align: /usr/lib/libvtkCharts.so.5.8.0
segment_translate_align: /usr/lib/libvtkViews.so.5.8.0
segment_translate_align: /usr/lib/libvtkInfovis.so.5.8.0
segment_translate_align: /usr/lib/libvtkWidgets.so.5.8.0
segment_translate_align: /usr/lib/libvtkHybrid.so.5.8.0
segment_translate_align: /usr/lib/libvtkParallel.so.5.8.0
segment_translate_align: /usr/lib/libvtkVolumeRendering.so.5.8.0
segment_translate_align: /usr/lib/libvtkRendering.so.5.8.0
segment_translate_align: /usr/lib/libvtkGraphics.so.5.8.0
segment_translate_align: /usr/lib/libvtkImaging.so.5.8.0
segment_translate_align: /usr/lib/libvtkIO.so.5.8.0
segment_translate_align: /usr/lib/libvtkFiltering.so.5.8.0
segment_translate_align: /usr/lib/libvtkCommon.so.5.8.0
segment_translate_align: /usr/lib/libvtksys.so.5.8.0
segment_translate_align: /usr/lib/libboost_system-mt.so
segment_translate_align: /usr/lib/libboost_filesystem-mt.so
segment_translate_align: /usr/lib/libboost_thread-mt.so
segment_translate_align: /usr/lib/libboost_date_time-mt.so
segment_translate_align: /usr/lib/libboost_iostreams-mt.so
segment_translate_align: /usr/lib/libboost_mpi-mt.so
segment_translate_align: /usr/lib/libboost_serialization-mt.so
segment_translate_align: /usr/local/lib/libpcl_common.so
segment_translate_align: /usr/lib/libflann_cpp_s.a
segment_translate_align: /usr/local/lib/libpcl_kdtree.so
segment_translate_align: /usr/local/lib/libpcl_octree.so
segment_translate_align: /usr/local/lib/libpcl_search.so
segment_translate_align: /usr/lib/libOpenNI.so
segment_translate_align: /usr/local/lib/libpcl_io.so
segment_translate_align: /usr/local/lib/libpcl_sample_consensus.so
segment_translate_align: /usr/local/lib/libpcl_filters.so
segment_translate_align: /usr/local/lib/libpcl_features.so
segment_translate_align: /usr/local/lib/libpcl_keypoints.so
segment_translate_align: /usr/local/lib/libpcl_ml.so
segment_translate_align: /usr/local/lib/libpcl_segmentation.so
segment_translate_align: /usr/local/lib/libpcl_visualization.so
segment_translate_align: /usr/local/lib/libpcl_outofcore.so
segment_translate_align: /usr/local/lib/libpcl_stereo.so
segment_translate_align: /usr/lib/libqhull.so
segment_translate_align: /usr/local/lib/libpcl_surface.so
segment_translate_align: /usr/local/lib/libpcl_registration.so
segment_translate_align: /usr/local/lib/libpcl_recognition.so
segment_translate_align: /usr/local/lib/libpcl_tracking.so
segment_translate_align: /usr/local/lib/libpcl_apps.so
segment_translate_align: /usr/local/lib/libpcl_people.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libGLU.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libGL.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libSM.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libICE.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libX11.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libXext.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libGLEW.so
segment_translate_align: /usr/local/lib/libpcl_simulation.so
segment_translate_align: /usr/lib/libboost_system-mt.so
segment_translate_align: /usr/lib/libboost_filesystem-mt.so
segment_translate_align: /usr/lib/libboost_thread-mt.so
segment_translate_align: /usr/lib/libboost_date_time-mt.so
segment_translate_align: /usr/lib/libboost_iostreams-mt.so
segment_translate_align: /usr/lib/libboost_mpi-mt.so
segment_translate_align: /usr/lib/libboost_serialization-mt.so
segment_translate_align: /usr/local/lib/libpcl_common.so
segment_translate_align: /usr/lib/libflann_cpp_s.a
segment_translate_align: /usr/local/lib/libpcl_kdtree.so
segment_translate_align: /usr/local/lib/libpcl_octree.so
segment_translate_align: /usr/local/lib/libpcl_search.so
segment_translate_align: /usr/lib/libOpenNI.so
segment_translate_align: /usr/local/lib/libpcl_io.so
segment_translate_align: /usr/local/lib/libpcl_sample_consensus.so
segment_translate_align: /usr/local/lib/libpcl_filters.so
segment_translate_align: /usr/local/lib/libpcl_features.so
segment_translate_align: /usr/local/lib/libpcl_keypoints.so
segment_translate_align: /usr/local/lib/libpcl_ml.so
segment_translate_align: /usr/local/lib/libpcl_segmentation.so
segment_translate_align: /usr/local/lib/libpcl_visualization.so
segment_translate_align: /usr/local/lib/libpcl_outofcore.so
segment_translate_align: /usr/local/lib/libpcl_stereo.so
segment_translate_align: /usr/lib/libqhull.so
segment_translate_align: /usr/local/lib/libpcl_surface.so
segment_translate_align: /usr/local/lib/libpcl_registration.so
segment_translate_align: /usr/local/lib/libpcl_recognition.so
segment_translate_align: /usr/local/lib/libpcl_tracking.so
segment_translate_align: /usr/local/lib/libpcl_apps.so
segment_translate_align: /usr/local/lib/libpcl_people.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libGLU.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libGL.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libSM.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libICE.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libX11.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libXext.so
segment_translate_align: /usr/lib/x86_64-linux-gnu/libGLEW.so
segment_translate_align: /usr/local/lib/libpcl_simulation.so
segment_translate_align: CMakeFiles/segment_translate_align.dir/build.make
segment_translate_align: CMakeFiles/segment_translate_align.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable segment_translate_align"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/segment_translate_align.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/segment_translate_align.dir/build: segment_translate_align
.PHONY : CMakeFiles/segment_translate_align.dir/build

CMakeFiles/segment_translate_align.dir/requires: CMakeFiles/segment_translate_align.dir/segment_translate_align.cpp.o.requires
.PHONY : CMakeFiles/segment_translate_align.dir/requires

CMakeFiles/segment_translate_align.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/segment_translate_align.dir/cmake_clean.cmake
.PHONY : CMakeFiles/segment_translate_align.dir/clean

CMakeFiles/segment_translate_align.dir/depend:
	cd /home/taylor/src/pcl/examples/segment_align/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/taylor/src/pcl/examples/segment_align /home/taylor/src/pcl/examples/segment_align /home/taylor/src/pcl/examples/segment_align/build /home/taylor/src/pcl/examples/segment_align/build /home/taylor/src/pcl/examples/segment_align/build/CMakeFiles/segment_translate_align.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/segment_translate_align.dir/depend

