FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_Navegacion/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/Modulo_Navegacion/msg/__init__.py"
  "src/Modulo_Navegacion/msg/_msg_laser.py"
  "src/Modulo_Navegacion/msg/_msg_habilitacion_modulo.py"
  "src/Modulo_Navegacion/msg/_msg_gest_navegacion.py"
  "src/Modulo_Navegacion/msg/_msg_modo.py"
  "src/Modulo_Navegacion/msg/_msg_waypoint.py"
  "src/Modulo_Navegacion/msg/_msg_gps.py"
  "src/Modulo_Navegacion/msg/_msg_errores.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
