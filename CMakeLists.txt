cmake_minimum_required(VERSION 2.8.3)
project(fyp)

find_package(catkin REQUIRED COMPONENTS
	roscpp)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

add_executable(fyp_node src/main.cpp)

target_link_libraries(fyp_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
