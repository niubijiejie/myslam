project(direct)
cmake_minimum_required(VERSION 2.8)
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块以使用g2o
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV REQUIRED )
find_package( G2O REQUIRED )


find_package(PCL 1.7 REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

link_directories(${PCL_LIBRARY_DIRS})

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    "/usr/include/eigen3/"
)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries( ${PROJECT_NAME}
   ${OpenCV_LIBS}
   g2o_core g2o_stuff g2o_solver_csparse g2o_types_sba g2o_csparse_extension
   ${PCL_LIBRARIES}
)
