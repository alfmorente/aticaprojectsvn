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
CMAKE_SOURCE_DIR = /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_PositionOrientation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_PositionOrientation

# Utility rule file for ROSBUILD_gensrv_py.

# Include the progress variables for this target.
include CMakeFiles/ROSBUILD_gensrv_py.dir/progress.make

CMakeFiles/ROSBUILD_gensrv_py: src/CITIUS_Control_PositionOrientation/srv/__init__.py

src/CITIUS_Control_PositionOrientation/srv/__init__.py: src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py
	$(CMAKE_COMMAND) -E cmake_progress_report /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_PositionOrientation/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating src/CITIUS_Control_PositionOrientation/srv/__init__.py"
	/opt/ros/groovy/share/rospy/rosbuild/scripts/gensrv_py.py --initpy /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_PositionOrientation/srv/srv_nodeStatus.srv

src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py: srv/srv_nodeStatus.srv
src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py: /opt/ros/groovy/share/rospy/rosbuild/scripts/gensrv_py.py
src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py: /opt/ros/groovy/share/roslib/cmake/../../../lib/roslib/gendeps
src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py: manifest.xml
src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py: /opt/ros/groovy/share/cpp_common/package.xml
src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py: /opt/ros/groovy/share/rostime/package.xml
src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py: /opt/ros/groovy/share/roscpp_traits/package.xml
src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py: /opt/ros/groovy/share/roscpp_serialization/package.xml
src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py: /opt/ros/groovy/share/genmsg/package.xml
src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py: /opt/ros/groovy/share/genpy/package.xml
src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py: /opt/ros/groovy/share/message_runtime/package.xml
src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py: /opt/ros/groovy/share/rosconsole/package.xml
src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py: /opt/ros/groovy/share/std_msgs/package.xml
src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py: /opt/ros/groovy/share/rosgraph_msgs/package.xml
src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py: /opt/ros/groovy/share/xmlrpcpp/package.xml
src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py: /opt/ros/groovy/share/roscpp/package.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_PositionOrientation/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py"
	/opt/ros/groovy/share/rospy/rosbuild/scripts/gensrv_py.py --noinitpy /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_PositionOrientation/srv/srv_nodeStatus.srv

ROSBUILD_gensrv_py: CMakeFiles/ROSBUILD_gensrv_py
ROSBUILD_gensrv_py: src/CITIUS_Control_PositionOrientation/srv/__init__.py
ROSBUILD_gensrv_py: src/CITIUS_Control_PositionOrientation/srv/_srv_nodeStatus.py
ROSBUILD_gensrv_py: CMakeFiles/ROSBUILD_gensrv_py.dir/build.make
.PHONY : ROSBUILD_gensrv_py

# Rule to build all files generated by this target.
CMakeFiles/ROSBUILD_gensrv_py.dir/build: ROSBUILD_gensrv_py
.PHONY : CMakeFiles/ROSBUILD_gensrv_py.dir/build

CMakeFiles/ROSBUILD_gensrv_py.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ROSBUILD_gensrv_py.dir/clean

CMakeFiles/ROSBUILD_gensrv_py.dir/depend:
	cd /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_PositionOrientation && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_PositionOrientation /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_PositionOrientation /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_PositionOrientation /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_PositionOrientation /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_PositionOrientation/CMakeFiles/ROSBUILD_gensrv_py.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ROSBUILD_gensrv_py.dir/depend
