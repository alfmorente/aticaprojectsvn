FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/CITIUS_Control_FrontCamera/msg"
  "src/CITIUS_Control_FrontCamera/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/CITIUS_Control_FrontCamera/srv/__init__.py"
  "src/CITIUS_Control_FrontCamera/srv/_srv_nodeStatus.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
