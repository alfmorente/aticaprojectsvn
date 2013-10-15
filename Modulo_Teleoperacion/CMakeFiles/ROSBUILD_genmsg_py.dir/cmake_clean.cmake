FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_Teleoperacion/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/Modulo_Teleoperacion/msg/__init__.py"
  "src/Modulo_Teleoperacion/msg/_msg_laser.py"
  "src/Modulo_Teleoperacion/msg/_msg_habilitacion_modulo.py"
  "src/Modulo_Teleoperacion/msg/_msg_modo.py"
  "src/Modulo_Teleoperacion/msg/_msg_com_teleoperado.py"
  "src/Modulo_Teleoperacion/msg/_msg_errores.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
