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
CMAKE_SOURCE_DIR = /home/hwj/desktop/myslam/pcl/testopenni/testopenni

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hwj/desktop/myslam/pcl/testopenni/testopenni/build

# Include any dependencies generated for this target.
include CMakeFiles/testopenni.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/testopenni.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/testopenni.dir/flags.make

CMakeFiles/testopenni.dir/main.cpp.o: CMakeFiles/testopenni.dir/flags.make
CMakeFiles/testopenni.dir/main.cpp.o: ../main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/hwj/desktop/myslam/pcl/testopenni/testopenni/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/testopenni.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/testopenni.dir/main.cpp.o -c /home/hwj/desktop/myslam/pcl/testopenni/testopenni/main.cpp

CMakeFiles/testopenni.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testopenni.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/hwj/desktop/myslam/pcl/testopenni/testopenni/main.cpp > CMakeFiles/testopenni.dir/main.cpp.i

CMakeFiles/testopenni.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testopenni.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/hwj/desktop/myslam/pcl/testopenni/testopenni/main.cpp -o CMakeFiles/testopenni.dir/main.cpp.s

CMakeFiles/testopenni.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/testopenni.dir/main.cpp.o.requires

CMakeFiles/testopenni.dir/main.cpp.o.provides: CMakeFiles/testopenni.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/testopenni.dir/build.make CMakeFiles/testopenni.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/testopenni.dir/main.cpp.o.provides

CMakeFiles/testopenni.dir/main.cpp.o.provides.build: CMakeFiles/testopenni.dir/main.cpp.o

# Object files for target testopenni
testopenni_OBJECTS = \
"CMakeFiles/testopenni.dir/main.cpp.o"

# External object files for target testopenni
testopenni_EXTERNAL_OBJECTS =

testopenni: CMakeFiles/testopenni.dir/main.cpp.o
testopenni: CMakeFiles/testopenni.dir/build.make
testopenni: /usr/lib/libOpenNI2.so
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libboost_system.so
testopenni: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
testopenni: /usr/lib/x86_64-linux-gnu/libboost_thread.so
testopenni: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
testopenni: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
testopenni: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
testopenni: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
testopenni: /usr/lib/x86_64-linux-gnu/libpthread.so
testopenni: /usr/lib/libpcl_common.so
testopenni: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
testopenni: /usr/lib/libpcl_kdtree.so
testopenni: /usr/lib/libpcl_octree.so
testopenni: /usr/lib/libpcl_search.so
testopenni: /usr/lib/x86_64-linux-gnu/libqhull.so
testopenni: /usr/lib/libpcl_surface.so
testopenni: /usr/lib/libpcl_sample_consensus.so
testopenni: /usr/lib/libOpenNI.so
testopenni: /usr/lib/libOpenNI2.so
testopenni: /usr/lib/libvtkCommon.so.5.8.0
testopenni: /usr/lib/libvtkFiltering.so.5.8.0
testopenni: /usr/lib/libvtkImaging.so.5.8.0
testopenni: /usr/lib/libvtkGraphics.so.5.8.0
testopenni: /usr/lib/libvtkGenericFiltering.so.5.8.0
testopenni: /usr/lib/libvtkIO.so.5.8.0
testopenni: /usr/lib/libvtkRendering.so.5.8.0
testopenni: /usr/lib/libvtkVolumeRendering.so.5.8.0
testopenni: /usr/lib/libvtkHybrid.so.5.8.0
testopenni: /usr/lib/libvtkWidgets.so.5.8.0
testopenni: /usr/lib/libvtkParallel.so.5.8.0
testopenni: /usr/lib/libvtkInfovis.so.5.8.0
testopenni: /usr/lib/libvtkGeovis.so.5.8.0
testopenni: /usr/lib/libvtkViews.so.5.8.0
testopenni: /usr/lib/libvtkCharts.so.5.8.0
testopenni: /usr/lib/libpcl_io.so
testopenni: /usr/lib/libpcl_filters.so
testopenni: /usr/lib/libpcl_features.so
testopenni: /usr/lib/libpcl_keypoints.so
testopenni: /usr/lib/libpcl_registration.so
testopenni: /usr/lib/libpcl_segmentation.so
testopenni: /usr/lib/libpcl_recognition.so
testopenni: /usr/lib/libpcl_visualization.so
testopenni: /usr/lib/libpcl_people.so
testopenni: /usr/lib/libpcl_outofcore.so
testopenni: /usr/lib/libpcl_tracking.so
testopenni: /usr/lib/libpcl_apps.so
testopenni: /usr/lib/x86_64-linux-gnu/libboost_system.so
testopenni: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
testopenni: /usr/lib/x86_64-linux-gnu/libboost_thread.so
testopenni: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
testopenni: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
testopenni: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
testopenni: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
testopenni: /usr/lib/x86_64-linux-gnu/libpthread.so
testopenni: /usr/lib/x86_64-linux-gnu/libqhull.so
testopenni: /usr/lib/libOpenNI.so
testopenni: /usr/lib/libOpenNI2.so
testopenni: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
testopenni: /usr/lib/libvtkCommon.so.5.8.0
testopenni: /usr/lib/libvtkFiltering.so.5.8.0
testopenni: /usr/lib/libvtkImaging.so.5.8.0
testopenni: /usr/lib/libvtkGraphics.so.5.8.0
testopenni: /usr/lib/libvtkGenericFiltering.so.5.8.0
testopenni: /usr/lib/libvtkIO.so.5.8.0
testopenni: /usr/lib/libvtkRendering.so.5.8.0
testopenni: /usr/lib/libvtkVolumeRendering.so.5.8.0
testopenni: /usr/lib/libvtkHybrid.so.5.8.0
testopenni: /usr/lib/libvtkWidgets.so.5.8.0
testopenni: /usr/lib/libvtkParallel.so.5.8.0
testopenni: /usr/lib/libvtkInfovis.so.5.8.0
testopenni: /usr/lib/libvtkGeovis.so.5.8.0
testopenni: /usr/lib/libvtkViews.so.5.8.0
testopenni: /usr/lib/libvtkCharts.so.5.8.0
testopenni: /usr/lib/libpcl_common.so
testopenni: /usr/lib/libpcl_kdtree.so
testopenni: /usr/lib/libpcl_octree.so
testopenni: /usr/lib/libpcl_search.so
testopenni: /usr/lib/libpcl_surface.so
testopenni: /usr/lib/libpcl_sample_consensus.so
testopenni: /usr/lib/libpcl_io.so
testopenni: /usr/lib/libpcl_filters.so
testopenni: /usr/lib/libpcl_features.so
testopenni: /usr/lib/libpcl_keypoints.so
testopenni: /usr/lib/libpcl_registration.so
testopenni: /usr/lib/libpcl_segmentation.so
testopenni: /usr/lib/libpcl_recognition.so
testopenni: /usr/lib/libpcl_visualization.so
testopenni: /usr/lib/libpcl_people.so
testopenni: /usr/lib/libpcl_outofcore.so
testopenni: /usr/lib/libpcl_tracking.so
testopenni: /usr/lib/libpcl_apps.so
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
testopenni: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
testopenni: /usr/lib/libvtkViews.so.5.8.0
testopenni: /usr/lib/libvtkInfovis.so.5.8.0
testopenni: /usr/lib/libvtkWidgets.so.5.8.0
testopenni: /usr/lib/libvtkVolumeRendering.so.5.8.0
testopenni: /usr/lib/libvtkHybrid.so.5.8.0
testopenni: /usr/lib/libvtkParallel.so.5.8.0
testopenni: /usr/lib/libvtkRendering.so.5.8.0
testopenni: /usr/lib/libvtkImaging.so.5.8.0
testopenni: /usr/lib/libvtkGraphics.so.5.8.0
testopenni: /usr/lib/libvtkIO.so.5.8.0
testopenni: /usr/lib/libvtkFiltering.so.5.8.0
testopenni: /usr/lib/libvtkCommon.so.5.8.0
testopenni: /usr/lib/libvtksys.so.5.8.0
testopenni: CMakeFiles/testopenni.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable testopenni"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testopenni.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/testopenni.dir/build: testopenni
.PHONY : CMakeFiles/testopenni.dir/build

CMakeFiles/testopenni.dir/requires: CMakeFiles/testopenni.dir/main.cpp.o.requires
.PHONY : CMakeFiles/testopenni.dir/requires

CMakeFiles/testopenni.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/testopenni.dir/cmake_clean.cmake
.PHONY : CMakeFiles/testopenni.dir/clean

CMakeFiles/testopenni.dir/depend:
	cd /home/hwj/desktop/myslam/pcl/testopenni/testopenni/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hwj/desktop/myslam/pcl/testopenni/testopenni /home/hwj/desktop/myslam/pcl/testopenni/testopenni /home/hwj/desktop/myslam/pcl/testopenni/testopenni/build /home/hwj/desktop/myslam/pcl/testopenni/testopenni/build /home/hwj/desktop/myslam/pcl/testopenni/testopenni/build/CMakeFiles/testopenni.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/testopenni.dir/depend

