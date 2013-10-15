FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_Teleoperacion/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/Modulo_Teleoperacion/msg_laser.h"
  "msg_gen/cpp/include/Modulo_Teleoperacion/msg_habilitacion_modulo.h"
  "msg_gen/cpp/include/Modulo_Teleoperacion/msg_modo.h"
  "msg_gen/cpp/include/Modulo_Teleoperacion/msg_com_teleoperado.h"
  "msg_gen/cpp/include/Modulo_Teleoperacion/msg_errores.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
