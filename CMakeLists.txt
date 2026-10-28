cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(cluster_extraction)

find_package(PCL 1.10 REQUIRED COMPONENTS common io segmentation filters visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set (CMAKE_CXX_STANDARD 11)


add_executable(cluster_extraction src/cluster_extraction.cpp src/Renderer.cpp)
target_link_libraries(cluster_extraction ${PCL_LIBRARIES})
