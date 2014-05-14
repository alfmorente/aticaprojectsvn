FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/CITIUS_Control_Manager/msg"
  "../src/CITIUS_Control_Manager/srv"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/CITIUS_Control_Manager/msg_cameraStatus.h"
  "../msg_gen/cpp/include/CITIUS_Control_Manager/msg_command.h"
  "../msg_gen/cpp/include/CITIUS_Control_Manager/msg_vehicleInfo.h"
  "../msg_gen/cpp/include/CITIUS_Control_Manager/msg_electricInfo.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
