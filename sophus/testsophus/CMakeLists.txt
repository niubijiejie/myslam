project(testsophus)
cmake_minimum_required(VERSION 2.8)

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries( ${PROJECT_NAME} ${Sophus_LIBRARIES} )
