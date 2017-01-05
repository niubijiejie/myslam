project(result)
cmake_minimum_required(VERSION 2.8)

find_package(PCL 1.7 REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

link_directories(${PCL_LIBRARY_DIRS})

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

