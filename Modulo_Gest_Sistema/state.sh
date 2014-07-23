#!/bin/bash
#rosparam set state_module_remote 2
/bin/chmod 777 /dev/ttyS*
/opt/ros/groovy/bin/rosparam set state_module_navigation 2
#rosparam set state_module_error_management 2
#rosparam set state_module_communication 2
#/opt/ros/groovy/bin/rosparam set state_module_driving 2
/opt/ros/groovy/bin/rosparam set state_module_gps 2
/opt/ros/groovy/bin/rosparam set state_module_laser3D 2
/opt/ros/groovy/bin/rosparam set state_module_front_laser_1 2
/opt/ros/groovy/bin/rosparam set state_module_front_laser_2 2
#/opt/ros/groovy/bin/rosparam set state_module_camera 2
/opt/ros/groovy/bin/rosparam set state_module_human_localization 2
/opt/ros/groovy/bin/rosparam set state_module_beacon 2
/opt/ros/groovy/bin/rosparam set state_module_convoy 2
/opt/ros/groovy/bin/rosparam set state_module_range_data_fusion 2




