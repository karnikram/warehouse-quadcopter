cmake_minimum_required(VERSION 2.8.3)
project(fyp)

set(CMAKE_MODULE_PATH /usr/local/lib/cmake/)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
	roscpp
  cv_bridge
  mavros_msgs)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

add_executable(fyp_node src/main.cpp)
add_executable(position_visualizer src/visualizer.cpp)
add_executable(aruco_tracker src/aruco_tracker.cpp)

target_link_libraries(fyp_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )

target_link_libraries(position_visualizer
  ${catkin_LIBRARIES}
)

target_link_libraries(aruco_tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${aruco_LIBS}
)
