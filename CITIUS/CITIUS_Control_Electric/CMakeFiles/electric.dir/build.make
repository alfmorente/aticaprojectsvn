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
CMAKE_SOURCE_DIR = /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric

# Include any dependencies generated for this target.
include CMakeFiles/electric.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/electric.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/electric.dir/flags.make

CMakeFiles/electric.dir/src/main.cpp.o: CMakeFiles/electric.dir/flags.make
CMakeFiles/electric.dir/src/main.cpp.o: src/main.cpp
CMakeFiles/electric.dir/src/main.cpp.o: manifest.xml
CMakeFiles/electric.dir/src/main.cpp.o: /opt/ros/groovy/share/cpp_common/package.xml
CMakeFiles/electric.dir/src/main.cpp.o: /opt/ros/groovy/share/rostime/package.xml
CMakeFiles/electric.dir/src/main.cpp.o: /opt/ros/groovy/share/roscpp_traits/package.xml
CMakeFiles/electric.dir/src/main.cpp.o: /opt/ros/groovy/share/roscpp_serialization/package.xml
CMakeFiles/electric.dir/src/main.cpp.o: /opt/ros/groovy/share/genmsg/package.xml
CMakeFiles/electric.dir/src/main.cpp.o: /opt/ros/groovy/share/genpy/package.xml
CMakeFiles/electric.dir/src/main.cpp.o: /opt/ros/groovy/share/message_runtime/package.xml
CMakeFiles/electric.dir/src/main.cpp.o: /opt/ros/groovy/share/rosconsole/package.xml
CMakeFiles/electric.dir/src/main.cpp.o: /opt/ros/groovy/share/std_msgs/package.xml
CMakeFiles/electric.dir/src/main.cpp.o: /opt/ros/groovy/share/rosgraph_msgs/package.xml
CMakeFiles/electric.dir/src/main.cpp.o: /opt/ros/groovy/share/xmlrpcpp/package.xml
CMakeFiles/electric.dir/src/main.cpp.o: /opt/ros/groovy/share/roscpp/package.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/electric.dir/src/main.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/electric.dir/src/main.cpp.o -c /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric/src/main.cpp

CMakeFiles/electric.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/electric.dir/src/main.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric/src/main.cpp > CMakeFiles/electric.dir/src/main.cpp.i

CMakeFiles/electric.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/electric.dir/src/main.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric/src/main.cpp -o CMakeFiles/electric.dir/src/main.cpp.s

CMakeFiles/electric.dir/src/main.cpp.o.requires:
.PHONY : CMakeFiles/electric.dir/src/main.cpp.o.requires

CMakeFiles/electric.dir/src/main.cpp.o.provides: CMakeFiles/electric.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/electric.dir/build.make CMakeFiles/electric.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/electric.dir/src/main.cpp.o.provides

CMakeFiles/electric.dir/src/main.cpp.o.provides.build: CMakeFiles/electric.dir/src/main.cpp.o

CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o: CMakeFiles/electric.dir/flags.make
CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o: src/ElectricConnectionManager.cpp
CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o: manifest.xml
CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o: /opt/ros/groovy/share/cpp_common/package.xml
CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o: /opt/ros/groovy/share/rostime/package.xml
CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o: /opt/ros/groovy/share/roscpp_traits/package.xml
CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o: /opt/ros/groovy/share/roscpp_serialization/package.xml
CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o: /opt/ros/groovy/share/genmsg/package.xml
CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o: /opt/ros/groovy/share/genpy/package.xml
CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o: /opt/ros/groovy/share/message_runtime/package.xml
CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o: /opt/ros/groovy/share/rosconsole/package.xml
CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o: /opt/ros/groovy/share/std_msgs/package.xml
CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o: /opt/ros/groovy/share/rosgraph_msgs/package.xml
CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o: /opt/ros/groovy/share/xmlrpcpp/package.xml
CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o: /opt/ros/groovy/share/roscpp/package.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o -c /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric/src/ElectricConnectionManager.cpp

CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric/src/ElectricConnectionManager.cpp > CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.i

CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric/src/ElectricConnectionManager.cpp -o CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.s

CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o.requires:
.PHONY : CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o.requires

CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o.provides: CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o.requires
	$(MAKE) -f CMakeFiles/electric.dir/build.make CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o.provides.build
.PHONY : CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o.provides

CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o.provides.build: CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o

CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o: CMakeFiles/electric.dir/flags.make
CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o: src/RosNode_Electric.cpp
CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o: manifest.xml
CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o: /opt/ros/groovy/share/cpp_common/package.xml
CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o: /opt/ros/groovy/share/rostime/package.xml
CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o: /opt/ros/groovy/share/roscpp_traits/package.xml
CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o: /opt/ros/groovy/share/roscpp_serialization/package.xml
CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o: /opt/ros/groovy/share/genmsg/package.xml
CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o: /opt/ros/groovy/share/genpy/package.xml
CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o: /opt/ros/groovy/share/message_runtime/package.xml
CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o: /opt/ros/groovy/share/rosconsole/package.xml
CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o: /opt/ros/groovy/share/std_msgs/package.xml
CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o: /opt/ros/groovy/share/rosgraph_msgs/package.xml
CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o: /opt/ros/groovy/share/xmlrpcpp/package.xml
CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o: /opt/ros/groovy/share/roscpp/package.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o -c /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric/src/RosNode_Electric.cpp

CMakeFiles/electric.dir/src/RosNode_Electric.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/electric.dir/src/RosNode_Electric.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric/src/RosNode_Electric.cpp > CMakeFiles/electric.dir/src/RosNode_Electric.cpp.i

CMakeFiles/electric.dir/src/RosNode_Electric.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/electric.dir/src/RosNode_Electric.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric/src/RosNode_Electric.cpp -o CMakeFiles/electric.dir/src/RosNode_Electric.cpp.s

CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o.requires:
.PHONY : CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o.requires

CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o.provides: CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o.requires
	$(MAKE) -f CMakeFiles/electric.dir/build.make CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o.provides.build
.PHONY : CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o.provides

CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o.provides.build: CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o

# Object files for target electric
electric_OBJECTS = \
"CMakeFiles/electric.dir/src/main.cpp.o" \
"CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o" \
"CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o"

# External object files for target electric
electric_EXTERNAL_OBJECTS =

bin/electric: CMakeFiles/electric.dir/src/main.cpp.o
bin/electric: CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o
bin/electric: CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o
bin/electric: CMakeFiles/electric.dir/build.make
bin/electric: CMakeFiles/electric.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/electric"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/electric.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/electric.dir/build: bin/electric
.PHONY : CMakeFiles/electric.dir/build

CMakeFiles/electric.dir/requires: CMakeFiles/electric.dir/src/main.cpp.o.requires
CMakeFiles/electric.dir/requires: CMakeFiles/electric.dir/src/ElectricConnectionManager.cpp.o.requires
CMakeFiles/electric.dir/requires: CMakeFiles/electric.dir/src/RosNode_Electric.cpp.o.requires
.PHONY : CMakeFiles/electric.dir/requires

CMakeFiles/electric.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/electric.dir/cmake_clean.cmake
.PHONY : CMakeFiles/electric.dir/clean

CMakeFiles/electric.dir/depend:
	cd /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Electric/CMakeFiles/electric.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/electric.dir/depend
