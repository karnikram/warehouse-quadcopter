
cmake_minimum_required(VERSION 2.8.3)
project(flyt)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS cpp_api)
find_package(Boost REQUIRED COMPONENTS system python)
find_package(PythonLibs 2.7 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

add_executable(navigation navigation.cpp)
target_link_libraries(navigation ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
