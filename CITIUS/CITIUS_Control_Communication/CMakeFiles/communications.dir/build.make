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
CMAKE_SOURCE_DIR = /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication

# Include any dependencies generated for this target.
include CMakeFiles/communications.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/communications.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/communications.dir/flags.make

CMakeFiles/communications.dir/src/main.cpp.o: CMakeFiles/communications.dir/flags.make
CMakeFiles/communications.dir/src/main.cpp.o: src/main.cpp
CMakeFiles/communications.dir/src/main.cpp.o: manifest.xml
CMakeFiles/communications.dir/src/main.cpp.o: /opt/ros/groovy/share/cpp_common/package.xml
CMakeFiles/communications.dir/src/main.cpp.o: /opt/ros/groovy/share/rostime/package.xml
CMakeFiles/communications.dir/src/main.cpp.o: /opt/ros/groovy/share/roscpp_traits/package.xml
CMakeFiles/communications.dir/src/main.cpp.o: /opt/ros/groovy/share/roscpp_serialization/package.xml
CMakeFiles/communications.dir/src/main.cpp.o: /opt/ros/groovy/share/genmsg/package.xml
CMakeFiles/communications.dir/src/main.cpp.o: /opt/ros/groovy/share/genpy/package.xml
CMakeFiles/communications.dir/src/main.cpp.o: /opt/ros/groovy/share/message_runtime/package.xml
CMakeFiles/communications.dir/src/main.cpp.o: /opt/ros/groovy/share/rosconsole/package.xml
CMakeFiles/communications.dir/src/main.cpp.o: /opt/ros/groovy/share/std_msgs/package.xml
CMakeFiles/communications.dir/src/main.cpp.o: /opt/ros/groovy/share/rosgraph_msgs/package.xml
CMakeFiles/communications.dir/src/main.cpp.o: /opt/ros/groovy/share/xmlrpcpp/package.xml
CMakeFiles/communications.dir/src/main.cpp.o: /opt/ros/groovy/share/roscpp/package.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/communications.dir/src/main.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/communications.dir/src/main.cpp.o -c /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication/src/main.cpp

CMakeFiles/communications.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/communications.dir/src/main.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication/src/main.cpp > CMakeFiles/communications.dir/src/main.cpp.i

CMakeFiles/communications.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/communications.dir/src/main.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication/src/main.cpp -o CMakeFiles/communications.dir/src/main.cpp.s

CMakeFiles/communications.dir/src/main.cpp.o.requires:
.PHONY : CMakeFiles/communications.dir/src/main.cpp.o.requires

CMakeFiles/communications.dir/src/main.cpp.o.provides: CMakeFiles/communications.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/communications.dir/build.make CMakeFiles/communications.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/communications.dir/src/main.cpp.o.provides

CMakeFiles/communications.dir/src/main.cpp.o.provides.build: CMakeFiles/communications.dir/src/main.cpp.o

CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o: CMakeFiles/communications.dir/flags.make
CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o: src/RosNode_Communications.cpp
CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o: manifest.xml
CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o: /opt/ros/groovy/share/cpp_common/package.xml
CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o: /opt/ros/groovy/share/rostime/package.xml
CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o: /opt/ros/groovy/share/roscpp_traits/package.xml
CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o: /opt/ros/groovy/share/roscpp_serialization/package.xml
CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o: /opt/ros/groovy/share/genmsg/package.xml
CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o: /opt/ros/groovy/share/genpy/package.xml
CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o: /opt/ros/groovy/share/message_runtime/package.xml
CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o: /opt/ros/groovy/share/rosconsole/package.xml
CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o: /opt/ros/groovy/share/std_msgs/package.xml
CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o: /opt/ros/groovy/share/rosgraph_msgs/package.xml
CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o: /opt/ros/groovy/share/xmlrpcpp/package.xml
CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o: /opt/ros/groovy/share/roscpp/package.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o -c /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication/src/RosNode_Communications.cpp

CMakeFiles/communications.dir/src/RosNode_Communications.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/communications.dir/src/RosNode_Communications.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication/src/RosNode_Communications.cpp > CMakeFiles/communications.dir/src/RosNode_Communications.cpp.i

CMakeFiles/communications.dir/src/RosNode_Communications.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/communications.dir/src/RosNode_Communications.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication/src/RosNode_Communications.cpp -o CMakeFiles/communications.dir/src/RosNode_Communications.cpp.s

CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o.requires:
.PHONY : CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o.requires

CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o.provides: CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o.requires
	$(MAKE) -f CMakeFiles/communications.dir/build.make CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o.provides.build
.PHONY : CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o.provides

CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o.provides.build: CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o

# Object files for target communications
communications_OBJECTS = \
"CMakeFiles/communications.dir/src/main.cpp.o" \
"CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o"

# External object files for target communications
communications_EXTERNAL_OBJECTS =

bin/communications: CMakeFiles/communications.dir/src/main.cpp.o
bin/communications: CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o
bin/communications: CMakeFiles/communications.dir/build.make
bin/communications: CMakeFiles/communications.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/communications"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/communications.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/communications.dir/build: bin/communications
.PHONY : CMakeFiles/communications.dir/build

CMakeFiles/communications.dir/requires: CMakeFiles/communications.dir/src/main.cpp.o.requires
CMakeFiles/communications.dir/requires: CMakeFiles/communications.dir/src/RosNode_Communications.cpp.o.requires
.PHONY : CMakeFiles/communications.dir/requires

CMakeFiles/communications.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/communications.dir/cmake_clean.cmake
.PHONY : CMakeFiles/communications.dir/clean

CMakeFiles/communications.dir/depend:
	cd /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication/CMakeFiles/communications.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/communications.dir/depend

