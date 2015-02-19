FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_Conduccion/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/Modulo_Conduccion/messageCAN.h"
  "msg_gen/cpp/include/Modulo_Conduccion/bomba.h"
  "msg_gen/cpp/include/Modulo_Conduccion/mastil.h"
  "msg_gen/cpp/include/Modulo_Conduccion/nivelBomba.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
