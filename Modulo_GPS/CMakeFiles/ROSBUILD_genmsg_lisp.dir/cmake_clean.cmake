FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_GPS/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "msg_gen/lisp/msg_stream.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_stream.lisp"
  "msg_gen/lisp/msg_module_enable.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_module_enable.lisp"
  "msg_gen/lisp/msg_backup.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_backup.lisp"
  "msg_gen/lisp/msg_gps.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_gps.lisp"
  "msg_gen/lisp/msg_error.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_error.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
