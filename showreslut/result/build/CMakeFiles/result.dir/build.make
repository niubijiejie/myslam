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

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hwj/desktop/myslam/showreslut/result

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hwj/desktop/myslam/showreslut/result/build

# Include any dependencies generated for this target.
include CMakeFiles/result.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/result.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/result.dir/flags.make

CMakeFiles/result.dir/main.cpp.o: CMakeFiles/result.dir/flags.make
CMakeFiles/result.dir/main.cpp.o: ../main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/hwj/desktop/myslam/showreslut/result/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/result.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/result.dir/main.cpp.o -c /home/hwj/desktop/myslam/showreslut/result/main.cpp

CMakeFiles/result.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/result.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/hwj/desktop/myslam/showreslut/result/main.cpp > CMakeFiles/result.dir/main.cpp.i

CMakeFiles/result.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/result.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/hwj/desktop/myslam/showreslut/result/main.cpp -o CMakeFiles/result.dir/main.cpp.s

CMakeFiles/result.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/result.dir/main.cpp.o.requires

CMakeFiles/result.dir/main.cpp.o.provides: CMakeFiles/result.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/result.dir/build.make CMakeFiles/result.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/result.dir/main.cpp.o.provides

CMakeFiles/result.dir/main.cpp.o.provides.build: CMakeFiles/result.dir/main.cpp.o

# Object files for target result
result_OBJECTS = \
"CMakeFiles/result.dir/main.cpp.o"

# External object files for target result
result_EXTERNAL_OBJECTS =

result: CMakeFiles/result.dir/main.cpp.o
result: CMakeFiles/result.dir/build.make
result: /usr/lib/x86_64-linux-gnu/libboost_system.so
result: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
result: /usr/lib/x86_64-linux-gnu/libboost_thread.so
result: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
result: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
result: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
result: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
result: /usr/lib/x86_64-linux-gnu/libpthread.so
result: /usr/lib/libpcl_common.so
result: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
result: /usr/lib/libpcl_kdtree.so
result: /usr/lib/libpcl_octree.so
result: /usr/lib/libpcl_search.so
result: /usr/lib/x86_64-linux-gnu/libqhull.so
result: /usr/lib/libpcl_surface.so
result: /usr/lib/libpcl_sample_consensus.so
result: /usr/lib/libOpenNI.so
result: /usr/lib/libOpenNI2.so
result: /usr/lib/libvtkCommon.so.5.8.0
result: /usr/lib/libvtkFiltering.so.5.8.0
result: /usr/lib/libvtkImaging.so.5.8.0
result: /usr/lib/libvtkGraphics.so.5.8.0
result: /usr/lib/libvtkGenericFiltering.so.5.8.0
result: /usr/lib/libvtkIO.so.5.8.0
result: /usr/lib/libvtkRendering.so.5.8.0
result: /usr/lib/libvtkVolumeRendering.so.5.8.0
result: /usr/lib/libvtkHybrid.so.5.8.0
result: /usr/lib/libvtkWidgets.so.5.8.0
result: /usr/lib/libvtkParallel.so.5.8.0
result: /usr/lib/libvtkInfovis.so.5.8.0
result: /usr/lib/libvtkGeovis.so.5.8.0
result: /usr/lib/libvtkViews.so.5.8.0
result: /usr/lib/libvtkCharts.so.5.8.0
result: /usr/lib/libpcl_io.so
result: /usr/lib/libpcl_filters.so
result: /usr/lib/libpcl_features.so
result: /usr/lib/libpcl_keypoints.so
result: /usr/lib/libpcl_registration.so
result: /usr/lib/libpcl_segmentation.so
result: /usr/lib/libpcl_recognition.so
result: /usr/lib/libpcl_visualization.so
result: /usr/lib/libpcl_people.so
result: /usr/lib/libpcl_outofcore.so
result: /usr/lib/libpcl_tracking.so
result: /usr/lib/libpcl_apps.so
result: /usr/lib/x86_64-linux-gnu/libboost_system.so
result: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
result: /usr/lib/x86_64-linux-gnu/libboost_thread.so
result: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
result: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
result: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
result: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
result: /usr/lib/x86_64-linux-gnu/libpthread.so
result: /usr/lib/x86_64-linux-gnu/libqhull.so
result: /usr/lib/libOpenNI.so
result: /usr/lib/libOpenNI2.so
result: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
result: /usr/lib/libvtkCommon.so.5.8.0
result: /usr/lib/libvtkFiltering.so.5.8.0
result: /usr/lib/libvtkImaging.so.5.8.0
result: /usr/lib/libvtkGraphics.so.5.8.0
result: /usr/lib/libvtkGenericFiltering.so.5.8.0
result: /usr/lib/libvtkIO.so.5.8.0
result: /usr/lib/libvtkRendering.so.5.8.0
result: /usr/lib/libvtkVolumeRendering.so.5.8.0
result: /usr/lib/libvtkHybrid.so.5.8.0
result: /usr/lib/libvtkWidgets.so.5.8.0
result: /usr/lib/libvtkParallel.so.5.8.0
result: /usr/lib/libvtkInfovis.so.5.8.0
result: /usr/lib/libvtkGeovis.so.5.8.0
result: /usr/lib/libvtkViews.so.5.8.0
result: /usr/lib/libvtkCharts.so.5.8.0
result: /usr/lib/libpcl_common.so
result: /usr/lib/libpcl_kdtree.so
result: /usr/lib/libpcl_octree.so
result: /usr/lib/libpcl_search.so
result: /usr/lib/libpcl_surface.so
result: /usr/lib/libpcl_sample_consensus.so
result: /usr/lib/libpcl_io.so
result: /usr/lib/libpcl_filters.so
result: /usr/lib/libpcl_features.so
result: /usr/lib/libpcl_keypoints.so
result: /usr/lib/libpcl_registration.so
result: /usr/lib/libpcl_segmentation.so
result: /usr/lib/libpcl_recognition.so
result: /usr/lib/libpcl_visualization.so
result: /usr/lib/libpcl_people.so
result: /usr/lib/libpcl_outofcore.so
result: /usr/lib/libpcl_tracking.so
result: /usr/lib/libpcl_apps.so
result: /usr/lib/libvtkViews.so.5.8.0
result: /usr/lib/libvtkInfovis.so.5.8.0
result: /usr/lib/libvtkWidgets.so.5.8.0
result: /usr/lib/libvtkVolumeRendering.so.5.8.0
result: /usr/lib/libvtkHybrid.so.5.8.0
result: /usr/lib/libvtkParallel.so.5.8.0
result: /usr/lib/libvtkRendering.so.5.8.0
result: /usr/lib/libvtkImaging.so.5.8.0
result: /usr/lib/libvtkGraphics.so.5.8.0
result: /usr/lib/libvtkIO.so.5.8.0
result: /usr/lib/libvtkFiltering.so.5.8.0
result: /usr/lib/libvtkCommon.so.5.8.0
result: /usr/lib/libvtksys.so.5.8.0
result: CMakeFiles/result.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable result"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/result.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/result.dir/build: result
.PHONY : CMakeFiles/result.dir/build

CMakeFiles/result.dir/requires: CMakeFiles/result.dir/main.cpp.o.requires
.PHONY : CMakeFiles/result.dir/requires

CMakeFiles/result.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/result.dir/cmake_clean.cmake
.PHONY : CMakeFiles/result.dir/clean

CMakeFiles/result.dir/depend:
	cd /home/hwj/desktop/myslam/showreslut/result/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hwj/desktop/myslam/showreslut/result /home/hwj/desktop/myslam/showreslut/result /home/hwj/desktop/myslam/showreslut/result/build /home/hwj/desktop/myslam/showreslut/result/build /home/hwj/desktop/myslam/showreslut/result/build/CMakeFiles/result.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/result.dir/depend

