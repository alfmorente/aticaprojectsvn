FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/CITIUS_Control_Communication/msg"
  "src/CITIUS_Control_Communication/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "srv_gen/cpp/include/CITIUS_Control_Communication/srv_zoomCommand.h"
  "srv_gen/cpp/include/CITIUS_Control_Communication/srv_zoomDirect.h"
  "srv_gen/cpp/include/CITIUS_Control_Communication/srv_dzoom.h"
  "srv_gen/cpp/include/CITIUS_Control_Communication/srv_focusDirect.h"
  "srv_gen/cpp/include/CITIUS_Control_Communication/srv_autofocusMode.h"
  "srv_gen/cpp/include/CITIUS_Control_Communication/srv_polarity.h"
  "srv_gen/cpp/include/CITIUS_Control_Communication/srv_vehicleStatus.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
