cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ud_mesh)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (ud_mesh ud_mesh.cpp)
target_link_libraries (ud_mesh ${PCL_LIBRARIES})

