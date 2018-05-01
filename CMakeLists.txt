cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(MYPROJECT)

find_package(PCL 1.7 REQUIRED)
FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
find_package(CUDA)

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_FLAGS_RELEASE "-O3")

#add_executable (planar_segmentation planar_segmentation.cpp)
#add_executable (find_keypoints find_keypoints.cpp)
#add_executable (find_features find_features.cpp)
cuda_add_executable (evaluation evaluation.cpp)

#target_link_libraries (planar_segmentation ${PCL_LIBRARIES})
#target_link_libraries (find_keypoints ${PCL_LIBRARIES})
#target_link_libraries (find_features ${PCL_LIBRARIES})
target_link_libraries (evaluation ${PCL_LIBRARIES}) #${Boost_LIBRARIES})
