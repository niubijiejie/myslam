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
CMAKE_SOURCE_DIR = /home/hwj/desktop/myslam/pcl/readimage/readimage

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hwj/desktop/myslam/pcl/readimage/readimage/build

# Include any dependencies generated for this target.
include CMakeFiles/readimage.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/readimage.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/readimage.dir/flags.make

CMakeFiles/readimage.dir/main.cpp.o: CMakeFiles/readimage.dir/flags.make
CMakeFiles/readimage.dir/main.cpp.o: ../main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/hwj/desktop/myslam/pcl/readimage/readimage/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/readimage.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/readimage.dir/main.cpp.o -c /home/hwj/desktop/myslam/pcl/readimage/readimage/main.cpp

CMakeFiles/readimage.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/readimage.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/hwj/desktop/myslam/pcl/readimage/readimage/main.cpp > CMakeFiles/readimage.dir/main.cpp.i

CMakeFiles/readimage.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/readimage.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/hwj/desktop/myslam/pcl/readimage/readimage/main.cpp -o CMakeFiles/readimage.dir/main.cpp.s

CMakeFiles/readimage.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/readimage.dir/main.cpp.o.requires

CMakeFiles/readimage.dir/main.cpp.o.provides: CMakeFiles/readimage.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/readimage.dir/build.make CMakeFiles/readimage.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/readimage.dir/main.cpp.o.provides

CMakeFiles/readimage.dir/main.cpp.o.provides.build: CMakeFiles/readimage.dir/main.cpp.o

# Object files for target readimage
readimage_OBJECTS = \
"CMakeFiles/readimage.dir/main.cpp.o"

# External object files for target readimage
readimage_EXTERNAL_OBJECTS =

readimage: CMakeFiles/readimage.dir/main.cpp.o
readimage: CMakeFiles/readimage.dir/build.make
readimage: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libboost_system.so
readimage: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
readimage: /usr/lib/x86_64-linux-gnu/libboost_thread.so
readimage: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
readimage: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
readimage: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
readimage: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
readimage: /usr/lib/x86_64-linux-gnu/libpthread.so
readimage: /usr/lib/libpcl_common.so
readimage: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
readimage: /usr/lib/libpcl_kdtree.so
readimage: /usr/lib/libpcl_octree.so
readimage: /usr/lib/libpcl_search.so
readimage: /usr/lib/x86_64-linux-gnu/libqhull.so
readimage: /usr/lib/libpcl_surface.so
readimage: /usr/lib/libpcl_sample_consensus.so
readimage: /usr/lib/libOpenNI.so
readimage: /usr/lib/libOpenNI2.so
readimage: /usr/lib/libvtkCommon.so.5.8.0
readimage: /usr/lib/libvtkFiltering.so.5.8.0
readimage: /usr/lib/libvtkImaging.so.5.8.0
readimage: /usr/lib/libvtkGraphics.so.5.8.0
readimage: /usr/lib/libvtkGenericFiltering.so.5.8.0
readimage: /usr/lib/libvtkIO.so.5.8.0
readimage: /usr/lib/libvtkRendering.so.5.8.0
readimage: /usr/lib/libvtkVolumeRendering.so.5.8.0
readimage: /usr/lib/libvtkHybrid.so.5.8.0
readimage: /usr/lib/libvtkWidgets.so.5.8.0
readimage: /usr/lib/libvtkParallel.so.5.8.0
readimage: /usr/lib/libvtkInfovis.so.5.8.0
readimage: /usr/lib/libvtkGeovis.so.5.8.0
readimage: /usr/lib/libvtkViews.so.5.8.0
readimage: /usr/lib/libvtkCharts.so.5.8.0
readimage: /usr/lib/libpcl_io.so
readimage: /usr/lib/libpcl_filters.so
readimage: /usr/lib/libpcl_features.so
readimage: /usr/lib/libpcl_keypoints.so
readimage: /usr/lib/libpcl_registration.so
readimage: /usr/lib/libpcl_segmentation.so
readimage: /usr/lib/libpcl_recognition.so
readimage: /usr/lib/libpcl_visualization.so
readimage: /usr/lib/libpcl_people.so
readimage: /usr/lib/libpcl_outofcore.so
readimage: /usr/lib/libpcl_tracking.so
readimage: /usr/lib/libpcl_apps.so
readimage: /usr/lib/x86_64-linux-gnu/libboost_system.so
readimage: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
readimage: /usr/lib/x86_64-linux-gnu/libboost_thread.so
readimage: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
readimage: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
readimage: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
readimage: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
readimage: /usr/lib/x86_64-linux-gnu/libpthread.so
readimage: /usr/lib/x86_64-linux-gnu/libqhull.so
readimage: /usr/lib/libOpenNI.so
readimage: /usr/lib/libOpenNI2.so
readimage: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
readimage: /usr/lib/libvtkCommon.so.5.8.0
readimage: /usr/lib/libvtkFiltering.so.5.8.0
readimage: /usr/lib/libvtkImaging.so.5.8.0
readimage: /usr/lib/libvtkGraphics.so.5.8.0
readimage: /usr/lib/libvtkGenericFiltering.so.5.8.0
readimage: /usr/lib/libvtkIO.so.5.8.0
readimage: /usr/lib/libvtkRendering.so.5.8.0
readimage: /usr/lib/libvtkVolumeRendering.so.5.8.0
readimage: /usr/lib/libvtkHybrid.so.5.8.0
readimage: /usr/lib/libvtkWidgets.so.5.8.0
readimage: /usr/lib/libvtkParallel.so.5.8.0
readimage: /usr/lib/libvtkInfovis.so.5.8.0
readimage: /usr/lib/libvtkGeovis.so.5.8.0
readimage: /usr/lib/libvtkViews.so.5.8.0
readimage: /usr/lib/libvtkCharts.so.5.8.0
readimage: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
readimage: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
readimage: /usr/lib/libpcl_common.so
readimage: /usr/lib/libpcl_kdtree.so
readimage: /usr/lib/libpcl_octree.so
readimage: /usr/lib/libpcl_search.so
readimage: /usr/lib/libpcl_surface.so
readimage: /usr/lib/libpcl_sample_consensus.so
readimage: /usr/lib/libpcl_io.so
readimage: /usr/lib/libpcl_filters.so
readimage: /usr/lib/libpcl_features.so
readimage: /usr/lib/libpcl_keypoints.so
readimage: /usr/lib/libpcl_registration.so
readimage: /usr/lib/libpcl_segmentation.so
readimage: /usr/lib/libpcl_recognition.so
readimage: /usr/lib/libpcl_visualization.so
readimage: /usr/lib/libpcl_people.so
readimage: /usr/lib/libpcl_outofcore.so
readimage: /usr/lib/libpcl_tracking.so
readimage: /usr/lib/libpcl_apps.so
readimage: /usr/lib/libvtkViews.so.5.8.0
readimage: /usr/lib/libvtkInfovis.so.5.8.0
readimage: /usr/lib/libvtkWidgets.so.5.8.0
readimage: /usr/lib/libvtkVolumeRendering.so.5.8.0
readimage: /usr/lib/libvtkHybrid.so.5.8.0
readimage: /usr/lib/libvtkParallel.so.5.8.0
readimage: /usr/lib/libvtkRendering.so.5.8.0
readimage: /usr/lib/libvtkImaging.so.5.8.0
readimage: /usr/lib/libvtkGraphics.so.5.8.0
readimage: /usr/lib/libvtkIO.so.5.8.0
readimage: /usr/lib/libvtkFiltering.so.5.8.0
readimage: /usr/lib/libvtkCommon.so.5.8.0
readimage: /usr/lib/libvtksys.so.5.8.0
readimage: CMakeFiles/readimage.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable readimage"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/readimage.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/readimage.dir/build: readimage
.PHONY : CMakeFiles/readimage.dir/build

CMakeFiles/readimage.dir/requires: CMakeFiles/readimage.dir/main.cpp.o.requires
.PHONY : CMakeFiles/readimage.dir/requires

CMakeFiles/readimage.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/readimage.dir/cmake_clean.cmake
.PHONY : CMakeFiles/readimage.dir/clean

CMakeFiles/readimage.dir/depend:
	cd /home/hwj/desktop/myslam/pcl/readimage/readimage/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hwj/desktop/myslam/pcl/readimage/readimage /home/hwj/desktop/myslam/pcl/readimage/readimage /home/hwj/desktop/myslam/pcl/readimage/readimage/build /home/hwj/desktop/myslam/pcl/readimage/readimage/build /home/hwj/desktop/myslam/pcl/readimage/readimage/build/CMakeFiles/readimage.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/readimage.dir/depend
