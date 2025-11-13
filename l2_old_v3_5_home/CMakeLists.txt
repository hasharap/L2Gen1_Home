cmake_minimum_required(VERSION 3.20)

project("l2_old_v3_5_home" C CXX ASM)

include(cmake/st-project.cmake)

add_executable(${PROJECT_NAME})
add_st_target_properties(${PROJECT_NAME})