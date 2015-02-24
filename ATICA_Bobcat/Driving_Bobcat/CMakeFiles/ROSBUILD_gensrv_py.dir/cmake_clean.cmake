FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/Driving_Bobcat/msg"
  "src/Driving_Bobcat/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/Driving_Bobcat/srv/__init__.py"
  "src/Driving_Bobcat/srv/_srv_vehicleStatus.py"
  "src/Driving_Bobcat/srv/_srv_nodeStatus.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
