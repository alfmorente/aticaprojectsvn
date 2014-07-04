FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_Navegacion/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/Modulo_Navegacion/msg_laser.h"
  "msg_gen/cpp/include/Modulo_Navegacion/msg_habilitacion_modulo.h"
  "msg_gen/cpp/include/Modulo_Navegacion/msg_gest_navegacion.h"
  "msg_gen/cpp/include/Modulo_Navegacion/msg_modo.h"
  "msg_gen/cpp/include/Modulo_Navegacion/msg_waypoint.h"
  "msg_gen/cpp/include/Modulo_Navegacion/msg_gps.h"
  "msg_gen/cpp/include/Modulo_Navegacion/msg_errores.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
