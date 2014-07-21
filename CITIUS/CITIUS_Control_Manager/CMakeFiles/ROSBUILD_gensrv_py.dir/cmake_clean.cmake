FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/CITIUS_Control_Manager/msg"
  "src/CITIUS_Control_Manager/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/CITIUS_Control_Manager/srv/__init__.py"
  "src/CITIUS_Control_Manager/srv/_srv_vehicleStatus.py"
  "src/CITIUS_Control_Manager/srv/_srv_nodeStatus.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
