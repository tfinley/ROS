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
CMAKE_SOURCE_DIR = /home/taylor/src/pcl/examples/translate_to_pick

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/taylor/src/pcl/examples/translate_to_pick/build

# Include any dependencies generated for this target.
include CMakeFiles/translate_to_pick.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/translate_to_pick.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/translate_to_pick.dir/flags.make

CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.o: CMakeFiles/translate_to_pick.dir/flags.make
CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.o: ../translate_to_pick.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/taylor/src/pcl/examples/translate_to_pick/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.o -c /home/taylor/src/pcl/examples/translate_to_pick/translate_to_pick.cpp

CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/taylor/src/pcl/examples/translate_to_pick/translate_to_pick.cpp > CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.i

CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/taylor/src/pcl/examples/translate_to_pick/translate_to_pick.cpp -o CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.s

CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.o.requires:
.PHONY : CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.o.requires

CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.o.provides: CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.o.requires
	$(MAKE) -f CMakeFiles/translate_to_pick.dir/build.make CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.o.provides.build
.PHONY : CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.o.provides

CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.o.provides.build: CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.o

# Object files for target translate_to_pick
translate_to_pick_OBJECTS = \
"CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.o"

# External object files for target translate_to_pick
translate_to_pick_EXTERNAL_OBJECTS =

translate_to_pick: CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.o
translate_to_pick: /usr/lib/libboost_system-mt.so
translate_to_pick: /usr/lib/libboost_filesystem-mt.so
translate_to_pick: /usr/lib/libboost_thread-mt.so
translate_to_pick: /usr/lib/libboost_date_time-mt.so
translate_to_pick: /usr/lib/libboost_iostreams-mt.so
translate_to_pick: /usr/lib/libboost_mpi-mt.so
translate_to_pick: /usr/lib/libboost_serialization-mt.so
translate_to_pick: /usr/local/lib/libpcl_common.so
translate_to_pick: /usr/lib/libflann_cpp_s.a
translate_to_pick: /usr/local/lib/libpcl_kdtree.so
translate_to_pick: /usr/local/lib/libpcl_octree.so
translate_to_pick: /usr/local/lib/libpcl_search.so
translate_to_pick: /usr/lib/libOpenNI.so
translate_to_pick: /usr/lib/libvtkCommon.so.5.8.0
translate_to_pick: /usr/lib/libvtkRendering.so.5.8.0
translate_to_pick: /usr/lib/libvtkHybrid.so.5.8.0
translate_to_pick: /usr/lib/libvtkCharts.so.5.8.0
translate_to_pick: /usr/local/lib/libpcl_io.so
translate_to_pick: /usr/local/lib/libpcl_sample_consensus.so
translate_to_pick: /usr/local/lib/libpcl_filters.so
translate_to_pick: /usr/local/lib/libpcl_features.so
translate_to_pick: /usr/local/lib/libpcl_keypoints.so
translate_to_pick: /usr/local/lib/libpcl_ml.so
translate_to_pick: /usr/local/lib/libpcl_segmentation.so
translate_to_pick: /usr/local/lib/libpcl_visualization.so
translate_to_pick: /usr/local/lib/libpcl_outofcore.so
translate_to_pick: /usr/local/lib/libpcl_stereo.so
translate_to_pick: /usr/lib/libqhull.so
translate_to_pick: /usr/local/lib/libpcl_surface.so
translate_to_pick: /usr/local/lib/libpcl_registration.so
translate_to_pick: /usr/local/lib/libpcl_recognition.so
translate_to_pick: /usr/local/lib/libpcl_tracking.so
translate_to_pick: /usr/local/lib/libpcl_apps.so
translate_to_pick: /usr/local/lib/libpcl_people.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libGLU.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libGL.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libSM.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libICE.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libX11.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libXext.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libGLEW.so
translate_to_pick: /usr/local/lib/libpcl_simulation.so
translate_to_pick: /usr/lib/libboost_system-mt.so
translate_to_pick: /usr/lib/libboost_filesystem-mt.so
translate_to_pick: /usr/lib/libboost_thread-mt.so
translate_to_pick: /usr/lib/libboost_date_time-mt.so
translate_to_pick: /usr/lib/libboost_iostreams-mt.so
translate_to_pick: /usr/lib/libboost_mpi-mt.so
translate_to_pick: /usr/lib/libboost_serialization-mt.so
translate_to_pick: /usr/lib/libqhull.so
translate_to_pick: /usr/lib/libOpenNI.so
translate_to_pick: /usr/lib/libflann_cpp_s.a
translate_to_pick: /usr/lib/libvtkCommon.so.5.8.0
translate_to_pick: /usr/lib/libvtkRendering.so.5.8.0
translate_to_pick: /usr/lib/libvtkHybrid.so.5.8.0
translate_to_pick: /usr/lib/libvtkCharts.so.5.8.0
translate_to_pick: /usr/lib/libvtkViews.so.5.8.0
translate_to_pick: /usr/lib/libvtkInfovis.so.5.8.0
translate_to_pick: /usr/lib/libvtkWidgets.so.5.8.0
translate_to_pick: /usr/lib/libvtkHybrid.so.5.8.0
translate_to_pick: /usr/lib/libvtkParallel.so.5.8.0
translate_to_pick: /usr/lib/libvtkVolumeRendering.so.5.8.0
translate_to_pick: /usr/lib/libvtkRendering.so.5.8.0
translate_to_pick: /usr/lib/libvtkGraphics.so.5.8.0
translate_to_pick: /usr/lib/libvtkImaging.so.5.8.0
translate_to_pick: /usr/lib/libvtkIO.so.5.8.0
translate_to_pick: /usr/lib/libvtkFiltering.so.5.8.0
translate_to_pick: /usr/lib/libvtkCommon.so.5.8.0
translate_to_pick: /usr/lib/libvtksys.so.5.8.0
translate_to_pick: /usr/lib/libboost_system-mt.so
translate_to_pick: /usr/lib/libboost_filesystem-mt.so
translate_to_pick: /usr/lib/libboost_thread-mt.so
translate_to_pick: /usr/lib/libboost_date_time-mt.so
translate_to_pick: /usr/lib/libboost_iostreams-mt.so
translate_to_pick: /usr/lib/libboost_mpi-mt.so
translate_to_pick: /usr/lib/libboost_serialization-mt.so
translate_to_pick: /usr/local/lib/libpcl_common.so
translate_to_pick: /usr/lib/libflann_cpp_s.a
translate_to_pick: /usr/local/lib/libpcl_kdtree.so
translate_to_pick: /usr/local/lib/libpcl_octree.so
translate_to_pick: /usr/local/lib/libpcl_search.so
translate_to_pick: /usr/lib/libOpenNI.so
translate_to_pick: /usr/local/lib/libpcl_io.so
translate_to_pick: /usr/local/lib/libpcl_sample_consensus.so
translate_to_pick: /usr/local/lib/libpcl_filters.so
translate_to_pick: /usr/local/lib/libpcl_features.so
translate_to_pick: /usr/local/lib/libpcl_keypoints.so
translate_to_pick: /usr/local/lib/libpcl_ml.so
translate_to_pick: /usr/local/lib/libpcl_segmentation.so
translate_to_pick: /usr/local/lib/libpcl_visualization.so
translate_to_pick: /usr/local/lib/libpcl_outofcore.so
translate_to_pick: /usr/local/lib/libpcl_stereo.so
translate_to_pick: /usr/lib/libqhull.so
translate_to_pick: /usr/local/lib/libpcl_surface.so
translate_to_pick: /usr/local/lib/libpcl_registration.so
translate_to_pick: /usr/local/lib/libpcl_recognition.so
translate_to_pick: /usr/local/lib/libpcl_tracking.so
translate_to_pick: /usr/local/lib/libpcl_apps.so
translate_to_pick: /usr/local/lib/libpcl_people.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libGLU.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libGL.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libSM.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libICE.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libX11.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libXext.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libGLEW.so
translate_to_pick: /usr/local/lib/libpcl_simulation.so
translate_to_pick: /usr/lib/libboost_system-mt.so
translate_to_pick: /usr/lib/libboost_filesystem-mt.so
translate_to_pick: /usr/lib/libboost_thread-mt.so
translate_to_pick: /usr/lib/libboost_date_time-mt.so
translate_to_pick: /usr/lib/libboost_iostreams-mt.so
translate_to_pick: /usr/lib/libboost_mpi-mt.so
translate_to_pick: /usr/lib/libboost_serialization-mt.so
translate_to_pick: /usr/local/lib/libpcl_common.so
translate_to_pick: /usr/lib/libflann_cpp_s.a
translate_to_pick: /usr/local/lib/libpcl_kdtree.so
translate_to_pick: /usr/local/lib/libpcl_octree.so
translate_to_pick: /usr/local/lib/libpcl_search.so
translate_to_pick: /usr/lib/libOpenNI.so
translate_to_pick: /usr/local/lib/libpcl_io.so
translate_to_pick: /usr/local/lib/libpcl_sample_consensus.so
translate_to_pick: /usr/local/lib/libpcl_filters.so
translate_to_pick: /usr/local/lib/libpcl_features.so
translate_to_pick: /usr/local/lib/libpcl_keypoints.so
translate_to_pick: /usr/local/lib/libpcl_ml.so
translate_to_pick: /usr/local/lib/libpcl_segmentation.so
translate_to_pick: /usr/local/lib/libpcl_visualization.so
translate_to_pick: /usr/local/lib/libpcl_outofcore.so
translate_to_pick: /usr/local/lib/libpcl_stereo.so
translate_to_pick: /usr/lib/libqhull.so
translate_to_pick: /usr/local/lib/libpcl_surface.so
translate_to_pick: /usr/local/lib/libpcl_registration.so
translate_to_pick: /usr/local/lib/libpcl_recognition.so
translate_to_pick: /usr/local/lib/libpcl_tracking.so
translate_to_pick: /usr/local/lib/libpcl_apps.so
translate_to_pick: /usr/local/lib/libpcl_people.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libGLU.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libGL.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libSM.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libICE.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libX11.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libXext.so
translate_to_pick: /usr/lib/x86_64-linux-gnu/libGLEW.so
translate_to_pick: /usr/local/lib/libpcl_simulation.so
translate_to_pick: CMakeFiles/translate_to_pick.dir/build.make
translate_to_pick: CMakeFiles/translate_to_pick.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable translate_to_pick"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/translate_to_pick.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/translate_to_pick.dir/build: translate_to_pick
.PHONY : CMakeFiles/translate_to_pick.dir/build

CMakeFiles/translate_to_pick.dir/requires: CMakeFiles/translate_to_pick.dir/translate_to_pick.cpp.o.requires
.PHONY : CMakeFiles/translate_to_pick.dir/requires

CMakeFiles/translate_to_pick.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/translate_to_pick.dir/cmake_clean.cmake
.PHONY : CMakeFiles/translate_to_pick.dir/clean

CMakeFiles/translate_to_pick.dir/depend:
	cd /home/taylor/src/pcl/examples/translate_to_pick/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/taylor/src/pcl/examples/translate_to_pick /home/taylor/src/pcl/examples/translate_to_pick /home/taylor/src/pcl/examples/translate_to_pick/build /home/taylor/src/pcl/examples/translate_to_pick/build /home/taylor/src/pcl/examples/translate_to_pick/build/CMakeFiles/translate_to_pick.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/translate_to_pick.dir/depend

