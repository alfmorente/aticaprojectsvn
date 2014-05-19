FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Common_files/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/Common_files/msg/__init__.py"
  "src/Common_files/msg/_msg_switch.py"
  "src/Common_files/msg/_msg_fcn_aux.py"
  "src/Common_files/msg/_msg_com_teleop.py"
  "src/Common_files/msg/_msg_emergency_stop.py"
  "src/Common_files/msg/_msg_rangedatafusion.py"
  "src/Common_files/msg/_msg_available.py"
  "src/Common_files/msg/_msg_backup.py"
  "src/Common_files/msg/_msg_waypoint.py"
  "src/Common_files/msg/_msg_mode.py"
  "src/Common_files/msg/_msg_module_enable.py"
  "src/Common_files/msg/_msg_error.py"
  "src/Common_files/msg/_msg_gps.py"
  "src/Common_files/msg/_msg_ctrl_camera.py"
  "src/Common_files/msg/_msg_stream.py"
  "src/Common_files/msg/_msg_laser.py"
  "src/Common_files/msg/_msg_camera.py"
  "src/Common_files/msg/_msg_info_stop.py"
  "src/Common_files/msg/_msg_navigation.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
