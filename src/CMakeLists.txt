cmake_minimum_required(VERSION 2.8.3)
project(vlo)

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  cv_bridge)

find_package(PCL 1.7 REQUIRED)
set(OpenCV_DIR /usr/local/opencv3/share/OpenCV)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS})
	



catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS
)


add_executable(laserhandle src/laserhandle.cpp)
target_link_libraries(laserhandle ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(lidtocam src/lidtocam.cpp)
target_link_libraries(lidtocam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CSPARSE_LIBRARY})

