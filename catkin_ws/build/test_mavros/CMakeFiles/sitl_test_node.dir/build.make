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
CMAKE_SOURCE_DIR = /home/hmcl/catkin_ws/src/mavros/test_mavros

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hmcl/catkin_ws/build/test_mavros

# Include any dependencies generated for this target.
include CMakeFiles/sitl_test_node.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sitl_test_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sitl_test_node.dir/flags.make

CMakeFiles/sitl_test_node.dir/sitl_test/sitl_test_node.cpp.o: CMakeFiles/sitl_test_node.dir/flags.make
CMakeFiles/sitl_test_node.dir/sitl_test/sitl_test_node.cpp.o: /home/hmcl/catkin_ws/src/mavros/test_mavros/sitl_test/sitl_test_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hmcl/catkin_ws/build/test_mavros/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sitl_test_node.dir/sitl_test/sitl_test_node.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sitl_test_node.dir/sitl_test/sitl_test_node.cpp.o -c /home/hmcl/catkin_ws/src/mavros/test_mavros/sitl_test/sitl_test_node.cpp

CMakeFiles/sitl_test_node.dir/sitl_test/sitl_test_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sitl_test_node.dir/sitl_test/sitl_test_node.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hmcl/catkin_ws/src/mavros/test_mavros/sitl_test/sitl_test_node.cpp > CMakeFiles/sitl_test_node.dir/sitl_test/sitl_test_node.cpp.i

CMakeFiles/sitl_test_node.dir/sitl_test/sitl_test_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sitl_test_node.dir/sitl_test/sitl_test_node.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hmcl/catkin_ws/src/mavros/test_mavros/sitl_test/sitl_test_node.cpp -o CMakeFiles/sitl_test_node.dir/sitl_test/sitl_test_node.cpp.s

# Object files for target sitl_test_node
sitl_test_node_OBJECTS = \
"CMakeFiles/sitl_test_node.dir/sitl_test/sitl_test_node.cpp.o"

# External object files for target sitl_test_node
sitl_test_node_EXTERNAL_OBJECTS =

/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: CMakeFiles/sitl_test_node.dir/sitl_test/sitl_test_node.cpp.o
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: CMakeFiles/sitl_test_node.dir/build.make
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /home/hmcl/catkin_ws/devel/.private/test_mavros/lib/libmavros_sitl_test.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/libcontrol_toolbox.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/librealtime_tools.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /home/hmcl/catkin_ws/devel/.private/mavros/lib/libmavros.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/x86_64-linux-gnu/libGeographic.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/libdiagnostic_updater.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/libeigen_conversions.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/liborocos-kdl.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /home/hmcl/catkin_ws/devel/.private/libmavconn/lib/libmavconn.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/libclass_loader.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/x86_64-linux-gnu/libPocoFoundation.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/x86_64-linux-gnu/libdl.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/libroslib.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/librospack.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/x86_64-linux-gnu/libpython3.8.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/libtf2_ros.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/libactionlib.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/libmessage_filters.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/libroscpp.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/librosconsole.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/libtf2.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/librostime.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /opt/ros/noetic/lib/libcpp_common.so
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node: CMakeFiles/sitl_test_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hmcl/catkin_ws/build/test_mavros/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sitl_test_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sitl_test_node.dir/build: /home/hmcl/catkin_ws/devel/.private/test_mavros/lib/test_mavros/sitl_test_node

.PHONY : CMakeFiles/sitl_test_node.dir/build

CMakeFiles/sitl_test_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sitl_test_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sitl_test_node.dir/clean

CMakeFiles/sitl_test_node.dir/depend:
	cd /home/hmcl/catkin_ws/build/test_mavros && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hmcl/catkin_ws/src/mavros/test_mavros /home/hmcl/catkin_ws/src/mavros/test_mavros /home/hmcl/catkin_ws/build/test_mavros /home/hmcl/catkin_ws/build/test_mavros /home/hmcl/catkin_ws/build/test_mavros/CMakeFiles/sitl_test_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sitl_test_node.dir/depend

