cmake_minimum_required(VERSION 3.10.2)
project(pcl_test)

#set (CMAKE_CXX_STANDARD 11)

#set(PCL_DIR //home/baoyuanjin/pcl-pcl-1.9.1)
find_package(PCL REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_test src/scaia_icp.cpp)

target_link_libraries (pcl_test ${PCL_LIBRARIES})

install(TARGETS pcl_test RUNTIME DESTINATION bin)