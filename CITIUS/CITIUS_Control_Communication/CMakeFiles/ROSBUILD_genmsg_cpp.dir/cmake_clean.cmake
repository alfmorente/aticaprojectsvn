FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/CITIUS_Control_Communication/msg"
  "src/CITIUS_Control_Communication/srv"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/CITIUS_Control_Communication/msg_ctrlFrontCamera.h"
  "msg_gen/cpp/include/CITIUS_Control_Communication/msg_electricInfo.h"
  "msg_gen/cpp/include/CITIUS_Control_Communication/msg_posOriInfo.h"
  "msg_gen/cpp/include/CITIUS_Control_Communication/msg_panTiltPosition.h"
  "msg_gen/cpp/include/CITIUS_Control_Communication/msg_frontCameraInfo.h"
  "msg_gen/cpp/include/CITIUS_Control_Communication/msg_irinfo.h"
  "msg_gen/cpp/include/CITIUS_Control_Communication/msg_tvinfo.h"
  "msg_gen/cpp/include/CITIUS_Control_Communication/msg_echoesFound.h"
  "msg_gen/cpp/include/CITIUS_Control_Communication/msg_command.h"
  "msg_gen/cpp/include/CITIUS_Control_Communication/msg_vehicleInfo.h"
  "msg_gen/cpp/include/CITIUS_Control_Communication/msg_electricCommand.h"
  "msg_gen/cpp/include/CITIUS_Control_Communication/msg_ctrlRearCamera.h"
  "msg_gen/cpp/include/CITIUS_Control_Communication/msg_rearCameraInfo.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
