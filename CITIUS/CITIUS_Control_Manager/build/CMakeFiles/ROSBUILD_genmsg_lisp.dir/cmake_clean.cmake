FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/CITIUS_Control_Manager/msg"
  "../src/CITIUS_Control_Manager/srv"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/msg_cameraStatus.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_msg_cameraStatus.lisp"
  "../msg_gen/lisp/msg_command.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_msg_command.lisp"
  "../msg_gen/lisp/msg_vehicleInfo.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_msg_vehicleInfo.lisp"
  "../msg_gen/lisp/msg_electricInfo.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_msg_electricInfo.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)