FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_Comunicaciones/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "msg_gen/lisp/msg_modo.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_modo.lisp"
  "msg_gen/lisp/msg_camaras.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_camaras.lisp"
  "msg_gen/lisp/msg_waypoint.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_waypoint.lisp"
  "msg_gen/lisp/msg_gps.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_gps.lisp"
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
