FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/Common_files/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/Common_files/msg_stream.h"
  "msg_gen/cpp/include/Common_files/msg_info_stop.h"
  "msg_gen/cpp/include/Common_files/msg_fcn_aux.h"
  "msg_gen/cpp/include/Common_files/msg_ctrl_camera.h"
  "msg_gen/cpp/include/Common_files/msg_switch.h"
  "msg_gen/cpp/include/Common_files/msg_waypoint.h"
  "msg_gen/cpp/include/Common_files/msg_rangedatafusion.h"
  "msg_gen/cpp/include/Common_files/msg_available.h"
  "msg_gen/cpp/include/Common_files/msg_module_enable.h"
  "msg_gen/cpp/include/Common_files/msg_backup.h"
  "msg_gen/cpp/include/Common_files/msg_gps.h"
  "msg_gen/cpp/include/Common_files/msg_camera.h"
  "msg_gen/cpp/include/Common_files/msg_com_teleop.h"
  "msg_gen/cpp/include/Common_files/msg_mode.h"
  "msg_gen/cpp/include/Common_files/msg_error.h"
  "msg_gen/cpp/include/Common_files/msg_laser.h"
  "msg_gen/cpp/include/Common_files/msg_navigation.h"
  "msg_gen/cpp/include/Common_files/msg_emergency_stop.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
