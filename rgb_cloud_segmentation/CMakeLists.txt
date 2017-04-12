cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(rgb_cloud_segmentation)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

add_executable(rgb_segment src/main.cpp)

target_link_libraries(rgb_segment ${PCL_LIBRARIES} ${OpenCV_LIBS})
