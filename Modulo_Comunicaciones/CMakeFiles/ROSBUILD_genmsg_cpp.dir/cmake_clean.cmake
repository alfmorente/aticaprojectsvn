FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_Comunicaciones/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/Modulo_Comunicaciones/msg_mode.h"
  "msg_gen/cpp/include/Modulo_Comunicaciones/msg_error.h"
  "msg_gen/cpp/include/Modulo_Comunicaciones/msg_gps.h"
  "msg_gen/cpp/include/Modulo_Comunicaciones/msg_com_teleoperate.h"
  "msg_gen/cpp/include/Modulo_Comunicaciones/msg_camera.h"
  "msg_gen/cpp/include/Modulo_Comunicaciones/msg_backup.h"
  "msg_gen/cpp/include/Modulo_Comunicaciones/msg_waypoints.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
