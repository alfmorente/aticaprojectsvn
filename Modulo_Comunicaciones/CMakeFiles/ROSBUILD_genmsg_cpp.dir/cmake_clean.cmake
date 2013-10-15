FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_Comunicaciones/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/Modulo_Comunicaciones/msg_modo.h"
  "msg_gen/cpp/include/Modulo_Comunicaciones/msg_camaras.h"
  "msg_gen/cpp/include/Modulo_Comunicaciones/msg_waypoint.h"
  "msg_gen/cpp/include/Modulo_Comunicaciones/msg_gps.h"
  "msg_gen/cpp/include/Modulo_Comunicaciones/msg_com_teleoperado.h"
  "msg_gen/cpp/include/Modulo_Comunicaciones/msg_errores.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
