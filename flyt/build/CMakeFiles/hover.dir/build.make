# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/flytpod/catkin_ws/src/fyp/flyt

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/flytpod/catkin_ws/src/fyp/flyt/build

# Include any dependencies generated for this target.
include CMakeFiles/hover.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/hover.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/hover.dir/flags.make

CMakeFiles/hover.dir/hover.cpp.o: CMakeFiles/hover.dir/flags.make
CMakeFiles/hover.dir/hover.cpp.o: ../hover.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/flytpod/catkin_ws/src/fyp/flyt/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/hover.dir/hover.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hover.dir/hover.cpp.o -c /home/flytpod/catkin_ws/src/fyp/flyt/hover.cpp

CMakeFiles/hover.dir/hover.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hover.dir/hover.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/flytpod/catkin_ws/src/fyp/flyt/hover.cpp > CMakeFiles/hover.dir/hover.cpp.i

CMakeFiles/hover.dir/hover.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hover.dir/hover.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/flytpod/catkin_ws/src/fyp/flyt/hover.cpp -o CMakeFiles/hover.dir/hover.cpp.s

CMakeFiles/hover.dir/hover.cpp.o.requires:

.PHONY : CMakeFiles/hover.dir/hover.cpp.o.requires

CMakeFiles/hover.dir/hover.cpp.o.provides: CMakeFiles/hover.dir/hover.cpp.o.requires
	$(MAKE) -f CMakeFiles/hover.dir/build.make CMakeFiles/hover.dir/hover.cpp.o.provides.build
.PHONY : CMakeFiles/hover.dir/hover.cpp.o.provides

CMakeFiles/hover.dir/hover.cpp.o.provides.build: CMakeFiles/hover.dir/hover.cpp.o


# Object files for target hover
hover_OBJECTS = \
"CMakeFiles/hover.dir/hover.cpp.o"

# External object files for target hover
hover_EXTERNAL_OBJECTS =

hover: CMakeFiles/hover.dir/hover.cpp.o
hover: CMakeFiles/hover.dir/build.make
hover: /flyt/flytos/flytcore/lib/libnavigation_bridge.so
hover: /flyt/flytos/flytcore/lib/libparam_bridge.so
hover: /opt/ros/kinetic/lib/libroscpp.so
hover: /usr/lib/arm-linux-gnueabihf/libboost_signals.so
hover: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
hover: /opt/ros/kinetic/lib/librosconsole.so
hover: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
hover: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
hover: /usr/lib/arm-linux-gnueabihf/liblog4cxx.so
hover: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
hover: /opt/ros/kinetic/lib/libxmlrpcpp.so
hover: /opt/ros/kinetic/lib/libroscpp_serialization.so
hover: /opt/ros/kinetic/lib/librostime.so
hover: /opt/ros/kinetic/lib/libcpp_common.so
hover: /usr/lib/arm-linux-gnueabihf/libboost_system.so
hover: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
hover: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
hover: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
hover: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
hover: /usr/lib/arm-linux-gnueabihf/libpthread.so
hover: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
hover: /opt/ros/kinetic/lib/libroslib.so
hover: /usr/lib/arm-linux-gnueabihf/libboost_system.so
hover: /usr/lib/arm-linux-gnueabihf/libboost_python.so
hover: /usr/lib/arm-linux-gnueabihf/libpython2.7.so
hover: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
hover: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
hover: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
hover: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
hover: /usr/lib/arm-linux-gnueabihf/libpthread.so
hover: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
hover: /opt/ros/kinetic/lib/libroslib.so
hover: /usr/lib/arm-linux-gnueabihf/libboost_python.so
hover: /usr/lib/arm-linux-gnueabihf/libpython2.7.so
hover: CMakeFiles/hover.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/flytpod/catkin_ws/src/fyp/flyt/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable hover"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hover.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/hover.dir/build: hover

.PHONY : CMakeFiles/hover.dir/build

CMakeFiles/hover.dir/requires: CMakeFiles/hover.dir/hover.cpp.o.requires

.PHONY : CMakeFiles/hover.dir/requires

CMakeFiles/hover.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/hover.dir/cmake_clean.cmake
.PHONY : CMakeFiles/hover.dir/clean

CMakeFiles/hover.dir/depend:
	cd /home/flytpod/catkin_ws/src/fyp/flyt/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/flytpod/catkin_ws/src/fyp/flyt /home/flytpod/catkin_ws/src/fyp/flyt /home/flytpod/catkin_ws/src/fyp/flyt/build /home/flytpod/catkin_ws/src/fyp/flyt/build /home/flytpod/catkin_ws/src/fyp/flyt/build/CMakeFiles/hover.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/hover.dir/depend

