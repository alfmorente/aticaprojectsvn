FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/CITIUS_Control_FrontCamera/msg"
  "src/CITIUS_Control_FrontCamera/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "srv_gen/cpp/include/CITIUS_Control_FrontCamera/srv_nodeStatus.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
