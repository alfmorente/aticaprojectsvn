# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera

# Include any dependencies generated for this target.
include CMakeFiles/frontCamera.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/frontCamera.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/frontCamera.dir/flags.make

CMakeFiles/frontCamera.dir/src/main.cpp.o: CMakeFiles/frontCamera.dir/flags.make
CMakeFiles/frontCamera.dir/src/main.cpp.o: src/main.cpp
CMakeFiles/frontCamera.dir/src/main.cpp.o: manifest.xml
CMakeFiles/frontCamera.dir/src/main.cpp.o: /opt/ros/groovy/share/cpp_common/package.xml
CMakeFiles/frontCamera.dir/src/main.cpp.o: /opt/ros/groovy/share/rostime/package.xml
CMakeFiles/frontCamera.dir/src/main.cpp.o: /opt/ros/groovy/share/roscpp_traits/package.xml
CMakeFiles/frontCamera.dir/src/main.cpp.o: /opt/ros/groovy/share/roscpp_serialization/package.xml
CMakeFiles/frontCamera.dir/src/main.cpp.o: /opt/ros/groovy/share/genmsg/package.xml
CMakeFiles/frontCamera.dir/src/main.cpp.o: /opt/ros/groovy/share/genpy/package.xml
CMakeFiles/frontCamera.dir/src/main.cpp.o: /opt/ros/groovy/share/message_runtime/package.xml
CMakeFiles/frontCamera.dir/src/main.cpp.o: /opt/ros/groovy/share/rosconsole/package.xml
CMakeFiles/frontCamera.dir/src/main.cpp.o: /opt/ros/groovy/share/std_msgs/package.xml
CMakeFiles/frontCamera.dir/src/main.cpp.o: /opt/ros/groovy/share/rosgraph_msgs/package.xml
CMakeFiles/frontCamera.dir/src/main.cpp.o: /opt/ros/groovy/share/xmlrpcpp/package.xml
CMakeFiles/frontCamera.dir/src/main.cpp.o: /opt/ros/groovy/share/roscpp/package.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/frontCamera.dir/src/main.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/frontCamera.dir/src/main.cpp.o -c /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera/src/main.cpp

CMakeFiles/frontCamera.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/frontCamera.dir/src/main.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera/src/main.cpp > CMakeFiles/frontCamera.dir/src/main.cpp.i

CMakeFiles/frontCamera.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/frontCamera.dir/src/main.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera/src/main.cpp -o CMakeFiles/frontCamera.dir/src/main.cpp.s

CMakeFiles/frontCamera.dir/src/main.cpp.o.requires:
.PHONY : CMakeFiles/frontCamera.dir/src/main.cpp.o.requires

CMakeFiles/frontCamera.dir/src/main.cpp.o.provides: CMakeFiles/frontCamera.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/frontCamera.dir/build.make CMakeFiles/frontCamera.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/frontCamera.dir/src/main.cpp.o.provides

CMakeFiles/frontCamera.dir/src/main.cpp.o.provides.build: CMakeFiles/frontCamera.dir/src/main.cpp.o

CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o: CMakeFiles/frontCamera.dir/flags.make
CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o: src/RosNode_FrontCamera.cpp
CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o: manifest.xml
CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o: /opt/ros/groovy/share/cpp_common/package.xml
CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o: /opt/ros/groovy/share/rostime/package.xml
CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o: /opt/ros/groovy/share/roscpp_traits/package.xml
CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o: /opt/ros/groovy/share/roscpp_serialization/package.xml
CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o: /opt/ros/groovy/share/genmsg/package.xml
CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o: /opt/ros/groovy/share/genpy/package.xml
CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o: /opt/ros/groovy/share/message_runtime/package.xml
CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o: /opt/ros/groovy/share/rosconsole/package.xml
CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o: /opt/ros/groovy/share/std_msgs/package.xml
CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o: /opt/ros/groovy/share/rosgraph_msgs/package.xml
CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o: /opt/ros/groovy/share/xmlrpcpp/package.xml
CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o: /opt/ros/groovy/share/roscpp/package.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o -c /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera/src/RosNode_FrontCamera.cpp

CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera/src/RosNode_FrontCamera.cpp > CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.i

CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera/src/RosNode_FrontCamera.cpp -o CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.s

CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o.requires:
.PHONY : CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o.requires

CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o.provides: CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o.requires
	$(MAKE) -f CMakeFiles/frontCamera.dir/build.make CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o.provides.build
.PHONY : CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o.provides

CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o.provides.build: CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o

CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o: CMakeFiles/frontCamera.dir/flags.make
CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o: src/AxisP3364LveDriver.cpp
CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o: manifest.xml
CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o: /opt/ros/groovy/share/cpp_common/package.xml
CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o: /opt/ros/groovy/share/rostime/package.xml
CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o: /opt/ros/groovy/share/roscpp_traits/package.xml
CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o: /opt/ros/groovy/share/roscpp_serialization/package.xml
CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o: /opt/ros/groovy/share/genmsg/package.xml
CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o: /opt/ros/groovy/share/genpy/package.xml
CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o: /opt/ros/groovy/share/message_runtime/package.xml
CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o: /opt/ros/groovy/share/rosconsole/package.xml
CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o: /opt/ros/groovy/share/std_msgs/package.xml
CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o: /opt/ros/groovy/share/rosgraph_msgs/package.xml
CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o: /opt/ros/groovy/share/xmlrpcpp/package.xml
CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o: /opt/ros/groovy/share/roscpp/package.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o -c /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera/src/AxisP3364LveDriver.cpp

CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera/src/AxisP3364LveDriver.cpp > CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.i

CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera/src/AxisP3364LveDriver.cpp -o CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.s

CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o.requires:
.PHONY : CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o.requires

CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o.provides: CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o.requires
	$(MAKE) -f CMakeFiles/frontCamera.dir/build.make CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o.provides.build
.PHONY : CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o.provides

CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o.provides.build: CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o

# Object files for target frontCamera
frontCamera_OBJECTS = \
"CMakeFiles/frontCamera.dir/src/main.cpp.o" \
"CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o" \
"CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o"

# External object files for target frontCamera
frontCamera_EXTERNAL_OBJECTS =

bin/frontCamera: CMakeFiles/frontCamera.dir/src/main.cpp.o
bin/frontCamera: CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o
bin/frontCamera: CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o
bin/frontCamera: CMakeFiles/frontCamera.dir/build.make
bin/frontCamera: CMakeFiles/frontCamera.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/frontCamera"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/frontCamera.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/frontCamera.dir/build: bin/frontCamera
.PHONY : CMakeFiles/frontCamera.dir/build

CMakeFiles/frontCamera.dir/requires: CMakeFiles/frontCamera.dir/src/main.cpp.o.requires
CMakeFiles/frontCamera.dir/requires: CMakeFiles/frontCamera.dir/src/RosNode_FrontCamera.cpp.o.requires
CMakeFiles/frontCamera.dir/requires: CMakeFiles/frontCamera.dir/src/AxisP3364LveDriver.cpp.o.requires
.PHONY : CMakeFiles/frontCamera.dir/requires

CMakeFiles/frontCamera.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/frontCamera.dir/cmake_clean.cmake
.PHONY : CMakeFiles/frontCamera.dir/clean

CMakeFiles/frontCamera.dir/depend:
	cd /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_FrontCamera/CMakeFiles/frontCamera.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/frontCamera.dir/depend

