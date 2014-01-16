FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_Comunicaciones/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/Modulo_Comunicaciones/msg/__init__.py"
  "src/Modulo_Comunicaciones/msg/_msg_mode.py"
  "src/Modulo_Comunicaciones/msg/_msg_error.py"
  "src/Modulo_Comunicaciones/msg/_msg_gps.py"
  "src/Modulo_Comunicaciones/msg/_msg_com_teleoperate.py"
  "src/Modulo_Comunicaciones/msg/_msg_camera.py"
  "src/Modulo_Comunicaciones/msg/_msg_backup.py"
  "src/Modulo_Comunicaciones/msg/_msg_waypoints.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
