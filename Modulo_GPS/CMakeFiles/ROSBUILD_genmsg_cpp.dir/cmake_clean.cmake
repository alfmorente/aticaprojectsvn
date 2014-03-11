FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Modulo_GPS/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/Modulo_GPS/msg_stream.h"
  "msg_gen/cpp/include/Modulo_GPS/msg_module_enable.h"
  "msg_gen/cpp/include/Modulo_GPS/msg_backup.h"
  "msg_gen/cpp/include/Modulo_GPS/msg_gps.h"
  "msg_gen/cpp/include/Modulo_GPS/msg_error.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
