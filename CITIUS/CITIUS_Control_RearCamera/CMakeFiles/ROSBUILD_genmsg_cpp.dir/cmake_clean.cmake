FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/CITIUS_Control_RearCamera/msg"
  "src/CITIUS_Control_RearCamera/srv"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/CITIUS_Control_RearCamera/msg_rearCameraInfo.h"
  "msg_gen/cpp/include/CITIUS_Control_RearCamera/msg_ctrlRearCamera.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
