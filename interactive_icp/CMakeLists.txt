cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(interactive_icp)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (interactive_icp.cpp interactive_icp.cpp)
target_link_libraries (interactive_icp.cpp ${PCL_LIBRARIES})
