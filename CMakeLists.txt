cmake_minimum_required(VERSION 3.10.2)
project(pcl_test)

#set (CMAKE_CXX_STANDARD 11)

#set(PCL_DIR //home/baoyuanjin/pcl-pcl-1.9.1)
find_package(PCL REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(scaia_icp src/scaia_icp.cpp)
target_link_libraries (scaia_icp ${PCL_LIBRARIES})
install(TARGETS scaia_icp RUNTIME DESTINATION bin)

add_executable(txt_to_pcd src/txt_to_pcd.cpp)
target_link_libraries (txt_to_pcd ${PCL_LIBRARIES})
install(TARGETS txt_to_pcd RUNTIME DESTINATION bin)

add_executable(pcd_rotate_pan src/pcd_rotate_pan.cpp)
target_link_libraries (pcd_rotate_pan ${PCL_LIBRARIES})
install(TARGETS pcd_rotate_pan RUNTIME DESTINATION bin)