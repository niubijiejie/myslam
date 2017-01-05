project(testimu6)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries(${PROJECT_NAME}  LpSensor.so)
    target_link_libraries(${PROJECT_NAME}  pthread)
    target_link_libraries(${PROJECT_NAME}  dl)
    target_link_libraries(${PROJECT_NAME}  rt)
