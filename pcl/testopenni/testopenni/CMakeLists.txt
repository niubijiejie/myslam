project(testopenni)
cmake_minimum_required(VERSION 2.8)
set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
MESSAGE(STATUS "The Opencv's include directory is:" ${OpenCV_INCLUDE_DIRS})



#OpenNI
FIND_PATH(OpenNI2_INCLUDE_DIRS OpenNI.h HINTS  $ENV{OPENNI2_INCLUDE} PATH_SUFFIXES openni2)
FIND_LIBRARY(OpenNI2_LIBRARY NAMES OpenNI2 HINTS  $ENV{OPENNI2_LIB} $ENV{OPENNI2_REDIST})
include_directories( ${OpenNI2_INCLUDE_DIRS} )
include_directories( "/usr/include/eigen3/" )


IF (OpenNI2_INCLUDE_DIRS AND OpenNI2_LIBRARY)
   SET(OpenNI2_FOUND TRUE)
ENDIF (OpenNI2_INCLUDE_DIRS AND OpenNI2_LIBRARY)

IF (OpenNI2_FOUND)
   # show which OpenNI2 was found only if not quiet
   SET(OpenNI2_LIBRARIES ${OpenNI2_LIBRARY})
   MESSAGE(STATUS "Found OpenNI2: ${OpenNI2_LIBRARIES}")
ELSE (OpenNI2_FOUND)
   # fatal error if OpenNI2 is required but not found
   IF (OpenNI2_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find OpenNI2. Environment variables OPENNI2_INCLUDE (directory containing OpenNI.h) and OPENNI2_LIB (directory containing OpenNI2 library) could bet set.")
   ENDIF (OpenNI2_FIND_REQUIRED)
ENDIF (OpenNI2_FOUND)
# pcl
find_package(PCL 1.7 REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

link_directories(${PCL_LIBRARY_DIRS})

#aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenNI2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
