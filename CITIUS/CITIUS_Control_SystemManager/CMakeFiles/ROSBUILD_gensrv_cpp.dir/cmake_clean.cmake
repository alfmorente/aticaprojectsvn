FILE(REMOVE_RECURSE
  "srv_gen"
  "srv_gen"
  "src/CITIUS_Control_SystemManager/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "srv_gen/cpp/include/CITIUS_Control_SystemManager/srv_nodeStatus.h"
  "srv_gen/cpp/include/CITIUS_Control_SystemManager/srv_vehicleStatus.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
