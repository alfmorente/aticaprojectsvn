FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/CITIUS_Control_Manager/msg"
  "src/CITIUS_Control_Manager/srv"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "srv_gen/lisp/srv_nodeStatus.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_srv_nodeStatus.lisp"
  "srv_gen/lisp/srv_vehicleStatus.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_srv_vehicleStatus.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
