FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_Teleoperacion/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "msg_gen/lisp/msg_laser.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_laser.lisp"
  "msg_gen/lisp/msg_habilitacion_modulo.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_habilitacion_modulo.lisp"
  "msg_gen/lisp/msg_modo.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_modo.lisp"
  "msg_gen/lisp/msg_com_teleoperado.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_com_teleoperado.lisp"
  "msg_gen/lisp/msg_errores.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_errores.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
