FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_HumanLocalization/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/Modulo_HumanLocalization/msg/__init__.py"
  "src/Modulo_HumanLocalization/msg/_msg_module_enable.py"
  "src/Modulo_HumanLocalization/msg/_msg_waypoint.py"
  "src/Modulo_HumanLocalization/msg/_msg_errores.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
