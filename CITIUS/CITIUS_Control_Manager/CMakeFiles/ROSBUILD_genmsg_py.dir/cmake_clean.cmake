FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/CITIUS_Control_Manager/msg"
  "src/CITIUS_Control_Manager/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/CITIUS_Control_Manager/msg/__init__.py"
  "src/CITIUS_Control_Manager/msg/_msg_switcher.py"
  "src/CITIUS_Control_Manager/msg/_msg_lastExec.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
