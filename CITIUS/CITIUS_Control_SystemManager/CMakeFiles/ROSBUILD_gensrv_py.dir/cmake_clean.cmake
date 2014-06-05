FILE(REMOVE_RECURSE
  "srv_gen"
  "srv_gen"
  "src/CITIUS_Control_SystemManager/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/CITIUS_Control_SystemManager/srv/__init__.py"
  "src/CITIUS_Control_SystemManager/srv/_srv_nodeStatus.py"
  "src/CITIUS_Control_SystemManager/srv/_srv_vehicleStatus.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
