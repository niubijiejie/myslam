project(testpangolin)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories( "/usr/include/eigen3" )

find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

#aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} main2.cpp)

target_link_libraries( ${PROJECT_NAME} ${Pangolin_LIBRARIES} )
