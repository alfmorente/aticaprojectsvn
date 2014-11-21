FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/CITIUS_Control_FrontCamera/msg"
  "src/CITIUS_Control_FrontCamera/srv"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "msg_gen/lisp/msg_ctrlFrontCamera.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_ctrlFrontCamera.lisp"
  "msg_gen/lisp/msg_frontCameraInfo.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_msg_frontCameraInfo.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
