FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_Conduccion/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/Modulo_Conduccion/msg/__init__.py"
  "src/Modulo_Conduccion/msg/_messageCAN.py"
  "src/Modulo_Conduccion/msg/_bomba.py"
  "src/Modulo_Conduccion/msg/_mastil.py"
  "src/Modulo_Conduccion/msg/_nivelBomba.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
