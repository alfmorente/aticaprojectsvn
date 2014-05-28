FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/CITIUS_Control_Driving/msg"
  "src/CITIUS_Control_Driving/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/CITIUS_Control_Driving/msg/__init__.py"
  "src/CITIUS_Control_Driving/msg/_msg_vehicleInformation.py"
  "src/CITIUS_Control_Driving/msg/_msg_command.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
