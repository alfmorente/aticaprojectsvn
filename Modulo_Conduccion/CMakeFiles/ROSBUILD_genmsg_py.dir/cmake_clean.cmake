FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_Conduccion/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/Modulo_Conduccion/msg/__init__.py"
  "src/Modulo_Conduccion/msg/_msg_switch.py"
  "src/Modulo_Conduccion/msg/_msg_com_teleop.py"
  "src/Modulo_Conduccion/msg/_msg_emergency_stop.py"
  "src/Modulo_Conduccion/msg/_msg_backup.py"
  "src/Modulo_Conduccion/msg/_msg_engine_break.py"
  "src/Modulo_Conduccion/msg/_msg_error.py"
  "src/Modulo_Conduccion/msg/_msg_info_stop.py"
  "src/Modulo_Conduccion/msg/_msg_navigation.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
