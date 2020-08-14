cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(testkeypoints)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (testkeypoints main.cpp src/overlap.cpp src/keypoints.cpp)
target_link_libraries (testkeypoints ${PCL_LIBRARIES})
