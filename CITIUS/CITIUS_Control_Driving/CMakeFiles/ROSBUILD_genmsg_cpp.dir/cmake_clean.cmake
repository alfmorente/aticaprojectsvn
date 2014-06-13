FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/CITIUS_Control_Driving/msg"
  "src/CITIUS_Control_Driving/srv"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/CITIUS_Control_Driving/msg_command.h"
  "msg_gen/cpp/include/CITIUS_Control_Driving/msg_vehicleInfo.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
