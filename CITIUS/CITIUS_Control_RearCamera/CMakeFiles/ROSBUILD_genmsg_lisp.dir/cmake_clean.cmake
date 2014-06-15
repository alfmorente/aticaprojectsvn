FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/CITIUS_Control_RearCamera/msg"
  "src/CITIUS_Control_RearCamera/srv"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "msg_gen/lisp/msg_rearCameraInfo.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_rearCameraInfo.lisp"
  "msg_gen/lisp/msg_ctrlRearCamera.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_ctrlRearCamera.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
