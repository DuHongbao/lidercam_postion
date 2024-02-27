cmake_minimum_required(VERSION 2.8.3)
project(camera_livox_merg)

SET(CMAKE_BUILD_TYPE "Debug")


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -std=c++0x -std=c++14 -fexceptions -Wno-unused-local-typedefs")

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  cv_bridge
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver2
  yolov8_ros_msgs
  )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
	${catkin_INCLUDE_DIRS} 
        ${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS})

 catkin_package(
   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
   DEPENDS EIGEN3 PCL OpenCV
  
 )

add_executable(camera_livox_merg camera_livox_merg.cpp)
add_executable(camera_livox_yolo camera_livox_yolo.cpp)
target_link_libraries(camera_livox_merg ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(camera_livox_yolo ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})


