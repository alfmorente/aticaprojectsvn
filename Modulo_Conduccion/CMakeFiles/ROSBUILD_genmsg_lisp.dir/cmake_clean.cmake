FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_Conduccion/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "msg_gen/lisp/msg_gest_navegacion.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_gest_navegacion.lisp"
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
