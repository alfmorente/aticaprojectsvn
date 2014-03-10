FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_Conduccion/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/Modulo_Conduccion/msg_switch.h"
  "msg_gen/cpp/include/Modulo_Conduccion/msg_com_teleop.h"
  "msg_gen/cpp/include/Modulo_Conduccion/msg_emergency_stop.h"
  "msg_gen/cpp/include/Modulo_Conduccion/msg_backup.h"
  "msg_gen/cpp/include/Modulo_Conduccion/msg_engine_break.h"
  "msg_gen/cpp/include/Modulo_Conduccion/msg_error.h"
  "msg_gen/cpp/include/Modulo_Conduccion/msg_info_stop.h"
  "msg_gen/cpp/include/Modulo_Conduccion/msg_navigation.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
