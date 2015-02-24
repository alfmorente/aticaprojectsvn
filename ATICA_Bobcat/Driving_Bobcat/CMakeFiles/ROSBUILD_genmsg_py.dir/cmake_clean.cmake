FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/Driving_Bobcat/msg"
  "src/Driving_Bobcat/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/Driving_Bobcat/msg/__init__.py"
  "src/Driving_Bobcat/msg/_msg_switcher.py"
  "src/Driving_Bobcat/msg/_msg_command.py"
  "src/Driving_Bobcat/msg/_msg_vehicleInfo.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
