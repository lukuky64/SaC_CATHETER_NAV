# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/lukuky64/catkin_ws/src/sensors_and_control/main

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lukuky64/catkin_ws/src/sensors_and_control/main/build

# Include any dependencies generated for this target.
include CMakeFiles/main_node.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main_node.dir/flags.make

CMakeFiles/main_node.dir/src/main.cpp.o: CMakeFiles/main_node.dir/flags.make
CMakeFiles/main_node.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lukuky64/catkin_ws/src/sensors_and_control/main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main_node.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main_node.dir/src/main.cpp.o -c /home/lukuky64/catkin_ws/src/sensors_and_control/main/src/main.cpp

CMakeFiles/main_node.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main_node.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lukuky64/catkin_ws/src/sensors_and_control/main/src/main.cpp > CMakeFiles/main_node.dir/src/main.cpp.i

CMakeFiles/main_node.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main_node.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lukuky64/catkin_ws/src/sensors_and_control/main/src/main.cpp -o CMakeFiles/main_node.dir/src/main.cpp.s

# Object files for target main_node
main_node_OBJECTS = \
"CMakeFiles/main_node.dir/src/main.cpp.o"

# External object files for target main_node
main_node_EXTERNAL_OBJECTS =

devel/lib/main/main_node: CMakeFiles/main_node.dir/src/main.cpp.o
devel/lib/main/main_node: CMakeFiles/main_node.dir/build.make
devel/lib/main/main_node: /opt/ros/noetic/lib/libroscpp.so
devel/lib/main/main_node: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/main/main_node: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
devel/lib/main/main_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
devel/lib/main/main_node: /opt/ros/noetic/lib/librosconsole.so
devel/lib/main/main_node: /opt/ros/noetic/lib/librosconsole_log4cxx.so
devel/lib/main/main_node: /opt/ros/noetic/lib/librosconsole_backend_interface.so
devel/lib/main/main_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/main/main_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
devel/lib/main/main_node: /opt/ros/noetic/lib/libxmlrpcpp.so
devel/lib/main/main_node: /opt/ros/noetic/lib/libroscpp_serialization.so
devel/lib/main/main_node: /opt/ros/noetic/lib/librostime.so
devel/lib/main/main_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
devel/lib/main/main_node: /opt/ros/noetic/lib/libcpp_common.so
devel/lib/main/main_node: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
devel/lib/main/main_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
devel/lib/main/main_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/main/main_node: CMakeFiles/main_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lukuky64/catkin_ws/src/sensors_and_control/main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable devel/lib/main/main_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main_node.dir/build: devel/lib/main/main_node

.PHONY : CMakeFiles/main_node.dir/build

CMakeFiles/main_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main_node.dir/clean

CMakeFiles/main_node.dir/depend:
	cd /home/lukuky64/catkin_ws/src/sensors_and_control/main/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lukuky64/catkin_ws/src/sensors_and_control/main /home/lukuky64/catkin_ws/src/sensors_and_control/main /home/lukuky64/catkin_ws/src/sensors_and_control/main/build /home/lukuky64/catkin_ws/src/sensors_and_control/main/build /home/lukuky64/catkin_ws/src/sensors_and_control/main/build/CMakeFiles/main_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main_node.dir/depend

