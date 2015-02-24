FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/Driving_Bobcat/msg"
  "src/Driving_Bobcat/srv"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/Driving_Bobcat/msg_switcher.h"
  "msg_gen/cpp/include/Driving_Bobcat/msg_command.h"
  "msg_gen/cpp/include/Driving_Bobcat/msg_vehicleInfo.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
