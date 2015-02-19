FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_Conduccion/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "msg_gen/lisp/messageCAN.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_messageCAN.lisp"
  "msg_gen/lisp/bomba.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_bomba.lisp"
  "msg_gen/lisp/mastil.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_mastil.lisp"
  "msg_gen/lisp/nivelBomba.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_nivelBomba.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
