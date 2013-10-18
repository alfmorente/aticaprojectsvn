FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_Laser2D/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/Modulo_Laser2D/msg/__init__.py"
  "src/Modulo_Laser2D/msg/_msg_laser.py"
  "src/Modulo_Laser2D/msg/_msg_errores.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
