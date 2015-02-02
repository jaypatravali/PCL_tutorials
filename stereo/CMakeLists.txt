cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(stereo)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(stereo stereocpp)
target_link_libraries(stereo ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})


